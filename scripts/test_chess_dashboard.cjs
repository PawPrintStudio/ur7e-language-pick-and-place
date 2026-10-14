// Offline regression checks for audience-facing state and request failures.
// Run: node scripts/test_chess_dashboard.cjs (no packages or robot needed).
const {readFileSync} = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const {test} = require('node:test');
const html = readFileSync(`${__dirname}/chess_dashboard.html`, 'utf8');
const source = html.match(/<script>([\s\S]*?)<\/script>/)[1].replace(/\npoll\(\);\s*$/, '');

function page(fetch = async () => ({ok:true, json:async () => ({})}), {operator = false} = {}) {
  const elements = new Map();
  const element = id => {
    if (!elements.has(id)) elements.set(id, {textContent:'', innerHTML:'', disabled:false, className:'',
      style:{vars:{}, setProperty(name, value) { this.vars[name] = value; }},
      setAttribute(name, value) { this[name] = value; }});
    return elements.get(id);
  };
  const classes = new Set();
  const context = vm.createContext({
    document: {getElementById:element, querySelectorAll:() => [], addEventListener:() => {},
      body:{classList:{add:x => classes.add(x), remove:x => classes.delete(x)}}},
    fetch, AbortSignal, setTimeout:() => {},
    location: {hash: operator ? '#operator' : '', search: ''},
  });
  vm.runInContext(source, context);
  return {element, classes, context, show:state => context.show(state)};
}

function boardState() {
  const squares = {};
  for (const f of 'abcdefgh') for (const r of '12345678') squares[f+r] = {s:true,o:false};
  return {fen:'8/8/8/8/8/8/8/8 w - - 0 1', live:{squares}};
}

test('selection links both boards, camera outline and raw thumbnail; unknown hides thumbnail', () => {
  const p = page(), state = boardState();
  state.live.squares.e2 = {s:true,o:true,w:true,t:40};
  state.live.heights = {p:40,k:80};
  state.live.camera = {width:100,height:100,bounds:[10,10,80,80],
    quads:{e2:[[10,10],[20,10],[20,20],[10,20]]},labels:[],crops:{e2:[10,5,10,15]}};
  p.show(state);
  p.context.selectSquare('e2');
  assert.match(p.element('piece-title').textContent, /Estimated pawn/);
  assert.match(p.element('piece-cutout').src, /^\/piece\?square=e2/);
  for (const id of ['record','seen']) assert.match(p.element(id).innerHTML, /stroke="#007d9e"/);
  assert.match(p.element('camera-overlay').innerHTML, /stroke="#28c8f0"/);
  p.element('cv-details').onclick();
  assert.doesNotMatch(p.element('camera-overlay').innerHTML, />40<\/text>/);
  assert.doesNotMatch(p.element('seen').innerHTML, />40<\/text>/);
  assert.doesNotMatch(p.element('piece-measure').textContent, /mm|height/);
  state.live.squares.e2.s = false;
  p.show(state);
  assert.equal(p.element('piece-cutout').hidden, true);
  assert.match(p.element('piece-title').textContent, /Unknown depth/);
  assert.match(p.element('camera-overlay').innerHTML, /fill="url\(#depth-hatch\)"/);
  p.context.cameraView(false);
  assert.equal(p.element('camera-overlay').viewBox, '0 0 100 100');
});

test('missing depth is unknown, never a complete board match', () => {
  const p = page(), state = boardState();
  delete state.live.squares.a1;
  p.show(state);
  assert.equal(p.element('agree').textContent, 'Some squares are unknown');
  assert.match(p.element('seen').innerHTML, /fill="url\(#hseen\)"/);
});

test('complete visible agreement and an occupied-square mismatch stay distinct', () => {
  const p = page(), state = boardState();
  p.show(state);
  assert.equal(p.element('agree').textContent, 'Visible positions agree');
  state.live.squares.e4 = {s:true,o:true,w:true,t:40};
  p.show(state);
  assert.equal(p.element('agree').textContent, '1 square differs');
  assert.match(p.element('headline').textContent, /Check e4/);
});

test('HTTP failure dims stale data and disables scan; recovery restores access', async () => {
  let ok = false;
  const p = page(async () => ({ok, json:async () => boardState()}));
  await p.context.poll();
  assert.equal(p.element('headline').textContent, 'Live updates paused');
  assert.equal(p.element('scan').disabled, true);
  assert.ok(p.classes.has('offline'));
  ok = true;
  await p.context.poll();
  assert.equal(p.element('scan').disabled, false);
  assert.equal(p.classes.has('offline'), false);
});

test('failed scan request explains the failure without an unhandled rejection', async () => {
  const p = page(async () => ({ok:false}));
  await p.element('scan').onclick();
  assert.match(p.element('scan-note').textContent, /could not be requested/);
});

test('failed robot stage keeps the stop message above turn instructions', () => {
  const p = page(), state = boardState();
  state.planned = {san:'e4',from:'e2',to:'e4'};
  state.stage = {name:'GRASP',outcome:'failed',reason:'grip check'};
  p.show(state);
  assert.match(p.element('headline').textContent, /Arm stopped/);
  assert.match(p.element('status').className, /bad/);
});

const PROMPT = {id:'p1', kind:'step', text:'b7 -> b5: go?', allow_text:false,
                options:[{key:'y',label:'Go',style:'go'},{key:'n',label:'Stop',style:'stop'}]};

test('the audience page never shows operator controls, whatever the state says', () => {
  const p = page(), state = boardState();
  state.prompt = PROMPT; state.prompt_age_s = 0.5;
  p.show(state);
  assert.equal(p.element('operator').hidden, true);
  assert.equal(p.element('op-buttons').innerHTML, '');
});

test('operator page: buttons follow the prompt, one click posts one answer with the prompt id', async () => {
  const posts = [];
  const p = page(async (url, init) => { posts.push([url, JSON.parse(init.body)]); return {ok:true, json:async () => ({})}; },
                 {operator:true});
  const state = boardState();
  state.prompt = PROMPT; state.prompt_age_s = 0.5;
  p.show(state);
  assert.equal(p.element('operator').hidden, false);
  assert.match(p.element('op-text').textContent, /b7 -> b5/);
  assert.match(p.element('op-buttons').innerHTML, /data-key="y"[^>]*>Go</);
  assert.doesNotMatch(p.element('op-buttons').innerHTML, /disabled/);
  assert.equal(p.element('op-form').hidden, true);
  await p.context.answer('y');
  await p.context.answer('y');                           // a double click is one answer
  assert.deepEqual(posts, [['/answer', {id:'p1', answer:'y'}]]);
  assert.match(p.element('op-buttons').innerHTML, /disabled/);
  assert.match(p.element('op-note').textContent, /Waiting for the game/);
  state.prompt = null;                                   // play took the answer
  p.show(state);
  assert.equal(p.element('op-buttons').innerHTML, '');
});

test('standing New game: answers the prompt with a game, resets the record without one; move list shows', async () => {
  const posts = [];
  const p = page(async (url, init) => { posts.push([url, init?.body ? JSON.parse(init.body) : null]); return {ok:true, json:async () => ({})}; },
                 {operator:true});
  const state = boardState();
  state.moves = ['e4', 'h6', 'd4']; state.arm = 'black';
  state.prompt = {...PROMPT, options:[...PROMPT.options, {key:'new', label:'New game'}, {key:'quit', label:'Quit'}]};
  state.prompt_age_s = 0.5;
  p.show(state);
  assert.equal(p.element('moves').innerHTML, '1. e4 <b>h6</b> 2. d4');
  assert.doesNotMatch(p.element('op-buttons').innerHTML, /data-key="new"/);   // the standing button covers it
  assert.equal(p.element('op-new').disabled, false);
  await p.element('op-new').onclick();
  assert.deepEqual(posts, [['/answer', {id:'p1', answer:'new'}]]);
  posts.length = 0;
  state.prompt = null; state.prompt_age_s = 1;                          // arm moving: the click is queued
  p.show(state);
  assert.equal(p.element('op-new').disabled, false);
  assert.match(p.element('op-standing-note').textContent, /after this step/);
  await p.element('op-new').onclick();
  assert.deepEqual(posts, [['/answer', {id:'*', answer:'new'}]]);
  posts.length = 0;
  state.prompt_age_s = null;                                             // no game at all: /reset
  p.show(state);
  assert.equal(p.element('op-new').disabled, false);
  await p.element('op-new').onclick();
  assert.deepEqual(posts, [['/reset', null]]);
  assert.match(p.element('op-note').textContent, /Record reset/);
});

test('a confirmation with a timeout shows the countdown; the operator view marks the body', () => {
  const p = page(undefined, {operator:true}), state = boardState();
  state.prompt = {id:'p9', kind:'confirm_move', text:'I saw e4. Right?', allow_text:true, timeout_s:5, default:'y',
                  options:[{key:'y',label:'Yes, e4',style:'go'},{key:'n',label:'No: read again'}]};
  state.prompt_age_s = 1.2;
  p.show(state);
  assert.match(p.element('op-count').textContent, /Yes, e4 in 4 s/);
  assert.ok(p.classes.has('operator'));
  state.prompt = {...state.prompt, timeout_s:null};
  p.show(state);
  assert.equal(p.element('op-count').textContent, '');
});

test('the character goes into the widest free margin around the board, sized to it, never on the board', () => {
  const p = page(), state = boardState();
  // The lab's close-up on 2026-10-14: board from 25 % to 73 % across, 29 % to 90 % down (frame 662 x 414).
  const quad = (x0, y0, x1, y1) => [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
  state.live.camera = {width:2000, height:1500, bounds:[725, 572, 662, 414],
    quads:{a1:quad(725 + 0.25 * 662, 572 + 0.29 * 414, 725 + 0.30 * 662, 572 + 0.35 * 414),
           h8:quad(725 + 0.68 * 662, 572 + 0.85 * 414, 725 + 0.73 * 662, 572 + 0.90 * 414)}, labels:[], crops:{}};
  const m = p.context.boardMargins(state.live.camera, true);
  assert.ok(Math.abs(m.left - 0.25) < 0.01 && Math.abs(m.right - 0.27) < 0.01 && Math.abs(m.top - 0.29) < 0.01 && Math.abs(m.bottom - 0.10) < 0.01);
  const top = p.context.robotPlacement(state.live.camera, true, 1200, 700);
  assert.equal(top.side, 'top');
  assert.ok(Math.abs(top.size - (0.29 * 700 - 24)) < 0.5, `size ${top.size}`);   // fits the 29 % strip
  p.show(state);
  assert.equal(p.element('robot')['data-side'], 'top');
  assert.match(p.element('robot').style.vars['--robot-h'], /^\d+px$/);
  // Whole-robot framing: the board is small in the middle; the margins are 45 % left, 40 % right,
  // 46 % above, 37 % below, so the strip above wins and the face is capped at 200 px tall.
  p.context.cameraView(false);
  const wide = p.context.robotPlacement(state.live.camera, false, 1200, 700);
  assert.equal(wide.side, 'top');
  assert.equal(wide.size, 200);
  // No camera data yet: the left corner, modest size.
  const none = p.context.robotPlacement(null, true, 1200, 700);
  assert.equal(none.side, 'left'); assert.equal(none.size, 276);
});

test('the cartoon arm shows a mood and a bubble while the voice speaks, fades after, thinks between', () => {
  const p = page(), state = boardState();
  state.voice = {event:'captured', line:'That piece is mine now.', age_s:0.5, seconds:3};
  p.show(state);
  assert.equal(p.element('robot')['data-mood'], 'smug');
  assert.match(p.element('robot').className, /speaking/);
  assert.equal(p.element('bubble').hidden, false);
  assert.equal(p.element('bubble-text').textContent, 'That piece is mine now.');
  state.voice.age_s = 8;                                   // said: bubble lingers, mouth still
  p.show(state);
  assert.doesNotMatch(p.element('robot').className, /speaking/);
  assert.match(p.element('bubble').className, /fading/);
  state.voice = null; state.thinking = true;
  p.show(state);
  assert.equal(p.element('robot')['data-mood'], 'thinking');
  assert.equal(p.element('bubble').hidden, true);
  state.thinking = false; state.voice = {event:'lost_game', line:'Well played.', age_s:1, seconds:2};
  p.show(state);
  assert.equal(p.element('robot')['data-mood'], 'sad');
});

test('operator page: a dead game greys the buttons out; typed moves only where allowed', () => {
  const p = page(undefined, {operator:true}), state = boardState();
  state.prompt = {...PROMPT, kind:'student_move', allow_text:true}; state.prompt_age_s = 30;
  p.show(state);
  assert.match(p.element('op-buttons').innerHTML, /disabled/);
  assert.equal(p.element('op-form').hidden, false);
  assert.equal(p.element('op-input').disabled, true);
  assert.match(p.element('op-alive').textContent, /no game running/);
  delete state.prompt; state.prompt_age_s = null;
  p.show(state);
  assert.match(p.element('op-text').textContent, /Start play in T3/);
});

test('audience headlines follow the prompt: stand clear, game over, your move', () => {
  const p = page(), state = boardState();
  state.fen = 'rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1'; state.arm = 'black';
  state.planned = {san:'b5',from:'b7',to:'b5'};
  state.prompt = {...PROMPT, kind:'clear'};
  p.show(state);
  assert.match(p.element('headline').textContent, /about to play b5/);
  assert.match(p.element('subline').textContent, /stand clear/);
  state.prompt = {...PROMPT, kind:'game_over'}; state.result = '1-0';
  p.show(state);
  assert.match(p.element('headline').textContent, /Game over · 1-0/);
  state.prompt = null; state.planned = null; state.result = null;
  state.fen = '8/8/8/8/8/8/8/8 w - - 0 1';              // matches the empty camera board
  p.show(state);
  assert.match(p.element('headline').textContent, /Your move · White/);
});
