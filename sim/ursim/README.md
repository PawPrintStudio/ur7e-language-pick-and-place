# URSim — sim tier 2

Universal Robots' own controller simulator (the real URControl software),
pinned to our robot's PolyScope version (5.23), with the lab's network
layout reproduced so commands here match commands there.

Full guide: [docs/SIMULATION.md](../../docs/SIMULATION.md). Short version:

```bash
./get_urcap.sh        # once
docker compose up -d
# PolyScope: http://localhost:6080/vnc.html
```
