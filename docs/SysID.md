# SysID

In this repo we use AdvantageKit for Logging infomation including the state and data sysID needs.

Here is what your mechinisim code will roughly look like.
```java
new SysIdRoutine(
    new SysIdRoutine.Config(
        null,
        null,
        null,
        (state) -> Logger.recordOutput("Intake/SysIdState", state.toString())),
    new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, rollers));
```

If you need to adjust the sysid settings (this is important since some mechinisms will destroy themselves on the default settings) look at this code:
```java
new SysIdRoutine(
    new SysIdRoutine.Config(
        Volts.per(Second).of(voltsPerSecond),
        Volts.of(volts),
        Second.of(seconds),
        (state) -> Logger.recordOutput("Intake/SysIdState", state.toString())),
    new SysIdRoutine.Mechanism((voltage) -> setVoltage(voltage.in(Volts)), null, rollers));
```
Reference the sysid docs or lsp for what each paramiter does.


You need to run through your four tests (quasistatic and dynamic in both dirrection) in order for the charactorization to give the needed data.

I would recomend trying to keep your mesurements someone consistent. For example I use Rotations and RPM for all the wheels.
In sysid when getting your values you need to ensure you are getting values for the correct type of PID controller. There is a significant difference between the REV pid and wpilib pid which needs to be accounted for.

If the resulting pid and feedforward values are not satisfactory or require additional tuning, you probably did something wrong.


