# Original programming in ROBOTIS R+ Task

Before this Python layer existed, the robot was programmed in **ROBOTIS R+ Task**,
ROBOTIS's graphical programming environment for its CM-series controllers.

![R+ Task debugging session](images/bioloid-cm530-rplus-task-debugging.jpg)

## How it worked

- **Task program** (`task 1 AMB` in the photo): the main control loop, running on the
  CM-530. It reads inputs from the remote controller and decides which behaviour to run.
- **Motion pages:** pre-recorded sequences of whole-body poses (keyframes) stored on the
  controller. The task program triggers a motion page by number.
- **Remote control:** button presses (arrow pad and numbered buttons) map to behaviours.
  During development, R+ Task's **Virtual Remote Controller** sent the same commands from
  the laptop.
- **Debugging:** the robot stays connected over USB (`usbserial` device in the photo) while
  the task runs, and the **Program Output Monitor** shows the program's live output, so
  logic and timing problems can be traced while the robot moves.

## Why I wrote the Python layer

| R+ Task | This repository |
|---|---|
| Graphical blocks, project files only readable in R+ Task | Plain Python and JSON, readable on GitHub and diffable in git |
| Motions edited in a GUI | Motions as JSON keyframes, recorded by hand with `teach.py` or written directly |
| Testing needs the physical robot | Simulator plus automated tests that run in CI on every push |
| Limited insight into servo health | Live voltage, temperature and load monitoring with warnings |

The two approaches work together: the motion pages and task logic on the controller can
stay as they are, while the PC tools are used for development, diagnostics and new motions.
