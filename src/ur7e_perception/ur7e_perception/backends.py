"""Interchangeable on-demand detectors; fixture mode is explicitly synthetic."""

import numpy as np
import cv2

from .core import Detection, PerceptionError


def foreground_mask(frame, bbox):
    """Depth foreground refinement for separated tabletop objects in dev mode.

    This is not NanoSAM and cannot separate touching equal-depth instances.
    No foreground/background depth separation means rejection, not a box mask.
    """
    h, w = frame.depth.shape
    x0, y0, x1, y1 = np.asarray(bbox, float)
    if not np.isfinite([x0, y0, x1, y1]).all():
        raise PerceptionError('non-finite box')
    x0, y0 = max(0, int(np.floor(x0))), max(0, int(np.floor(y0)))
    x1, y1 = min(w, int(np.ceil(x1))), min(h, int(np.ceil(y1)))
    if x1 <= x0 or y1 <= y0:
        raise PerceptionError('empty detection box')
    d = frame.depth[y0:y1, x0:x1]
    good = np.isfinite(d) & (d > 0.05)
    scene = frame.depth[np.isfinite(frame.depth) & (frame.depth > 0.05)]
    if not good.any() or not len(scene):
        raise PerceptionError('no depth for segmentation')
    table_depth = np.median(scene)
    near = np.percentile(d[good], 20)
    if table_depth - near < 0.008:
        raise PerceptionError('no separable depth foreground')
    candidate = (good & (d < table_depth-0.008) & (np.abs(d-near) < 0.025)).astype('uint8')
    count, components, stats, _ = cv2.connectedComponentsWithStats(candidate)
    if count <= 1:
        raise PerceptionError('empty foreground mask')
    sizes = stats[1:, cv2.CC_STAT_AREA]
    order = np.argsort(sizes)
    if len(order) > 1 and sizes[order[-2]] > 0.5 * sizes[order[-1]]:
        raise PerceptionError('ambiguous foreground instances')
    mask = np.zeros((h, w), dtype=np.uint8)
    mask[y0:y1, x0:x1] = components == (int(order[-1])+1)
    return mask


class ColorFixtureBackend:
    """Known-color block detector, only for geometry/transport regression."""

    def detect(self, frame, query):
        colors = {'red block': 0, 'green block': 1, 'blue block': 2}
        if query not in colors:
            raise PerceptionError('fixture supports only red, green, or blue block')
        channel = colors[query]
        rgb = frame.rgb.astype(float)
        mask = (rgb[:, :, channel] > 100)
        mask &= rgb[:, :, channel] > np.max(np.delete(rgb, channel, axis=2), axis=2)*1.6
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype('uint8'))
        candidates = [i for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] >= 20]
        if len(candidates) != 1:
            raise PerceptionError('target absent or ambiguous')
        i = candidates[0]
        x, y, w, h = stats[i, :4]
        return Detection((labels == i).astype('uint8'), (x, y, x+w, y+h), 1.0, query)


class Owlv2Backend:
    """Real open-vocabulary model plus depth-based development segmentation."""

    def __init__(self, model='google/owlv2-base-patch16-ensemble', device='cpu', threshold=0.15):
        import torch
        from transformers import Owlv2Processor, Owlv2ForObjectDetection
        self.torch, self.device, self.threshold = torch, device, threshold
        self.processor = Owlv2Processor.from_pretrained(model)
        self.model = Owlv2ForObjectDetection.from_pretrained(model).to(device).eval()

    def detect(self, frame, query):
        from PIL import Image
        inputs = self.processor(text=[[query]], images=Image.fromarray(frame.rgb), return_tensors='pt')
        inputs = {k: v.to(self.device) for k, v in inputs.items()}
        with self.torch.inference_mode():
            outputs = self.model(**inputs)
        result = self.processor.post_process_grounded_object_detection(
            outputs, target_sizes=[frame.rgb.shape[:2]], threshold=self.threshold)[0]
        if len(result['scores']) == 0:
            raise PerceptionError('target not detected')
        order = result['scores'].argsort(descending=True)
        i = int(order[0])
        bbox = tuple(result['boxes'][i].detach().cpu().tolist())
        for other in order[1:]:
            # Reject any distinct plausible target (duplicate proposals are OK).
            b = result['boxes'][int(other)].detach().cpu().numpy()
            a = np.array(bbox)
            overlap = np.maximum(0, np.minimum(a[2:], b[2:])-np.maximum(a[:2], b[:2])).prod()
            union = (a[2:]-a[:2]).prod()+(b[2:]-b[:2]).prod()-overlap
            if overlap/max(union, 1e-9) < 0.5:
                raise PerceptionError('multiple plausible targets; clarify query')
        return Detection(foreground_mask(frame, bbox), bbox, float(result['scores'][i]), query)


class NanoOwlBackend:
    """Jetson engine adapter; TensorRT engines are built on the target Jetson."""

    def __init__(self, owl_engine, sam_encoder, sam_decoder, threshold=0.15):
        from nanoowl.owl_predictor import OwlPredictor
        from nanosam.utils.predictor import Predictor
        self.owl = OwlPredictor('google/owlvit-base-patch32', image_encoder_engine=owl_engine)
        self.sam = Predictor(image_encoder=sam_encoder, mask_decoder=sam_decoder)
        self.threshold = threshold

    def detect(self, frame, query):
        from PIL import Image
        image = Image.fromarray(frame.rgb)
        output = self.owl.predict(image=image, text=[query], threshold=self.threshold)
        if len(output.scores) != 1:
            raise PerceptionError('NanoOWL target absent or ambiguous')
        box = output.boxes[0].detach().cpu().numpy()
        self.sam.set_image(image)
        mask, _, _ = self.sam.predict(np.array([box[:2], box[2:]]), np.array([2, 3]))
        mask = (mask[0, 0].detach().cpu().numpy() > 0).astype('uint8')
        if mask.shape != frame.depth.shape:
            raise PerceptionError('NanoSAM mask dimensions disagree with capture')
        return Detection(mask, tuple(box), float(output.scores[0]), query)


def make_backend(name, **kwargs):
    if name == 'fixture':
        return ColorFixtureBackend()
    if name == 'owlv2':
        return Owlv2Backend(**kwargs)
    if name == 'nanoowl':
        return NanoOwlBackend(**kwargs)
    raise PerceptionError(f'unknown backend: {name}')
