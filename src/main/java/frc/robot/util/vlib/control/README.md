# Vlib Control Classes

These control classes are drop in replacements for the stardard PID and Feedforward classes.

Some of these classes (mostly the Feedforwards) are mostly copy pastes of the classes from wpilib.
This is because some fields that need to be tunable are final or private.
As a result, if the behind the scenes logic ever changes in wpilib, it may need to be changed here as well.