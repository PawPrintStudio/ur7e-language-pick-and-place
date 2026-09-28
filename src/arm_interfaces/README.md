# arm_interfaces

Message and service definitions shared by the `arm_*` nodes. No code.

Currently:

| Interface | Used by |
|---|---|
| `msg/Command` | a validated manipulation command |
| `srv/ParseIntent` | `intent_parser` (task 3.1) |

## The concept: why interfaces get their own package

ROS 2 generates message code with `rosidl`, which only runs from an
**`ament_cmake`** package. Our nodes are Python (`ament_python`), and a package
can only be one build type. So the choice is: make every node that defines a
message a CMake package, or put all the definitions in one small CMake package
that the Python packages depend on.

The second is standard practice, and the reason is not just build mechanics:
**an interface is a contract between two nodes, so it belongs to neither.**
Putting `ParseIntent` inside `arm_language` would mean the orchestrator has to
depend on the whole language stack — the SDK, the corpus, the backends — in
order to know the shape of a reply. Here it depends on a package that is
nothing but shapes.

Expect this package to grow as the phases land: `DetectObject` and
`LocateGrasp` (phase 2), `ExecuteMotionPrimitive` and `GripperCommand`
(phase 1), `PickCommand` (phase 4) are all named in the architecture's node
graph.

## Working on it

Interface changes are ABI changes — every node that uses one must be rebuilt:

```bash
colcon build --packages-select arm_interfaces
colcon build   # then everything that depends on it
ros2 interface show arm_interfaces/srv/ParseIntent
```

Two things to know before editing a definition:

- **Constants go in the interface, not in each consumer.** `Command.msg`
  carries `ACTION_PICK`/`ACTION_PICK_AND_PLACE` so the closed vocabulary is
  visible to every language binding at once, instead of being re-declared (and
  eventually re-declared *differently*) in each node.
- **Order matters in `package.xml`.** Format 3 requires `member_of_group` to
  come after every `*_depend` tag; `ament_xmllint` fails the build otherwise.
  That one cost a CI round-trip.
