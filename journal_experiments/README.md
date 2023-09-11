# multi Robot
- one robot prints the path (printer)
- second robot moves to path point and back to drop something on the printed path (dropper)

## Printer
### Launch
```bash
roslaunch journal_experiments mur_sim.launch
roslaunch journal_experiments start_experiment.launch
```

wait for URs to reach home pose, then:

```bash
roslaunch journal_experiments twist_sim.launch
```

## Dropper
### Launch
Launch before printers start_experiment (needs path from printer)
```bash
roslaunch journal_experiments mur_dropper.launch
rosrun journal_experiments state_machine_dropper.py
rosrun journal_experiments dropper_publish_testdata.py
```