r"""
ROS 2 node wrapping :class:`~arm_language.parser.IntentParser`.

Deliberately thin. All of the thinking lives in the plain-Python modules; this
file only does the three things that genuinely need ROS: read parameters,
serve ``ParseIntent``, and log. Keeping it this thin is what lets the rest of
the package be tested without a ROS installation — and it means a bug is
almost never in here.

Run it::

    ros2 run arm_language intent_parser_node
    ros2 run arm_language intent_parser_node --ros-args -p backend:=keyword

Call it::

    ros2 service call /intent_parser/parse_intent \\
        arm_interfaces/srv/ParseIntent "{text: 'pick up the hammer'}"
"""
import json

import rclpy
from rclpy.node import Node

from arm_interfaces.srv import ParseIntent

from .backends import BackendError, available, create
from .guardrails import GuardrailPolicy
from .parser import IntentParser
from .result import Outcome
from .schema import MOTION_ACTIONS

_OUTCOME_TO_FIELD = {
    Outcome.ACCEPTED: ParseIntent.Response.OUTCOME_ACCEPTED,
    Outcome.NEEDS_CONFIRMATION: ParseIntent.Response.OUTCOME_NEEDS_CONFIRMATION,
    Outcome.REFUSED: ParseIntent.Response.OUTCOME_REFUSED,
}


class IntentParserNode(Node):
    """Serves ``~/parse_intent``."""

    def __init__(self) -> None:
        """Read parameters, build the parser, advertise the service."""
        super().__init__('intent_parser')

        self.declare_parameter('backend', 'claude')
        self.declare_parameter('model', '')
        self.declare_parameter('accept_threshold', 0.75)
        self.declare_parameter('confirm_threshold', 0.40)
        self.declare_parameter('require_confirmation', False)
        self.declare_parameter('allowed_actions', list(MOTION_ACTIONS))

        backend_name = self.get_parameter('backend').value
        model = self.get_parameter('model').value

        try:
            policy = GuardrailPolicy(
                allowed_actions=tuple(
                    self.get_parameter('allowed_actions').value),
                accept_threshold=self.get_parameter('accept_threshold').value,
                confirm_threshold=self.get_parameter('confirm_threshold').value,
                require_confirmation=self.get_parameter(
                    'require_confirmation').value,
            )
        except ValueError as exc:
            # A nonsensical policy is a configuration error, and the only safe
            # response is to refuse to start. Coming up with a silently
            # corrected policy would mean the robot's caution level is not what
            # the launch file says it is.
            self.get_logger().fatal(f'Invalid guardrail policy: {exc}')
            raise

        try:
            backend = create(backend_name, **({'model': model} if model else {}))
        except BackendError as exc:
            self.get_logger().fatal(
                f'Could not start backend "{backend_name}": {exc}\n'
                f'Available backends: {", ".join(available())}.'
            )
            raise

        self._parser = IntentParser(backend, policy)
        self._service = self.create_service(
            ParseIntent, '~/parse_intent', self._on_parse)

        describe = getattr(backend, 'describe', None)
        self.get_logger().info(
            f'intent_parser ready on {self._service.srv_name} — backend: '
            f'{describe() if describe else backend_name}'
        )
        self.get_logger().info(
            f'Guardrails: actions={",".join(policy.allowed_actions)} '
            f'accept>={policy.accept_threshold} '
            f'confirm>={policy.confirm_threshold} '
            f'require_confirmation={policy.require_confirmation}'
        )

    def _on_parse(self, request, response):
        """Handle one ParseIntent call."""
        result = self._parser.parse(request.text)

        response.outcome = _OUTCOME_TO_FIELD[result.outcome]
        response.reason_code = result.reason_code.value
        response.message = result.message

        if result.command is not None:
            response.command.action = result.command.action
            response.command.target_query = result.command.target_query
            response.command.place_target = result.command.place_target or ''
            response.command.modifiers_json = result.command.modifiers_json()
            response.command.confidence = float(result.command.confidence)

        # One structured line per parse. The orchestrator's run records
        # (architecture §1.2) need to answer "which stage failed and why"
        # months later, and `detail` already carries latency and backend.
        # The raw backend response is dropped: it can be long, and on a public
        # demo it is the user's own words echoed into a shared log.
        record = {
            'outcome': result.outcome.value,
            'reason_code': result.reason_code.value,
            'detail': {key: value for key, value in result.detail.items()
                       if key != 'raw_response'},
        }
        line = f'PARSE {json.dumps(record, default=str)}'
        if result.outcome is Outcome.REFUSED:
            self.get_logger().warning(line)
        else:
            self.get_logger().info(line)
        return response


def main(args=None) -> None:
    """Spin the node."""
    rclpy.init(args=args)
    node = None
    try:
        node = IntentParserNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
