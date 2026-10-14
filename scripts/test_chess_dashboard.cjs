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
  state.prompt = null; state.prompt_age_s = 1;                          // arm moving: nothing to answer
  p.show(state);
  assert.equal(p.element('op-new').disabled, true);
  assert.match(p.element('op-standing-note').textContent, /Wait for the arm/);
  state.prompt_age_s = null;                                             // no game at all: /reset
  p.show(state);
  assert.equal(p.element('op-new').disabled, false);
  await p.element('op-new').onclick();
  assert.deepEqual(posts, [['/reset', null]]);
  assert.match(p.element('op-note').textContent, /Record reset/);
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
