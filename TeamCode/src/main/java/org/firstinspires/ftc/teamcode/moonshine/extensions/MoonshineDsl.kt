package org.firstinspires.ftc.teamcode.moonshine.extensions

import org.firstinspires.ftc.teamcode.moonshine.Command
import org.firstinspires.ftc.teamcode.moonshine.SharedSubsystem
import org.firstinspires.ftc.teamcode.moonshine.Subsystem
import org.firstinspires.ftc.teamcode.moonshine.builtin.*
import java.util.function.Supplier

@DslMarker
annotation class MoonshineDsl

@MoonshineDsl
fun Command.run(command: Command) {
    this.children += command
}

@MoonshineDsl
fun Command.serial(inner: SerialCommand.() -> Unit) /*where T : @SupportsMultipleChildren Command*/ {
    this.children += SerialCommand().apply(inner)
}

@MoonshineDsl
fun Command.parallel(behavior: ParallelCommand.Behavior, inner: ParallelCommand.() -> Unit) {
    this.children += ParallelCommand(behavior).apply(inner)
}

@MoonshineDsl
fun Command.sleep(durationMs: Long) {
    this.children += SleepCommand(durationMs)
}

@MoonshineDsl
fun Command.switch(switchResult: Supplier<Int>, inner: SwitchCommand.() -> Unit) {
    this.children += SwitchCommand(switchResult).apply(inner)
}

@MoonshineDsl
fun Command.trigger(cond: Supplier<Boolean>, inner: TriggerCommand.() -> Unit) {
    this.children += TriggerCommand(cond).apply(inner)
}

@MoonshineDsl
fun Command.inline(func: () -> Unit) {
    this.children += InlineCommand(func)
}

@MoonshineDsl
fun newSerial(inner: SerialCommand.() -> Unit): SerialCommand {
    return SerialCommand().apply(inner)
}

@MoonshineDsl
fun newContinuous(inner: ContinuousReuseCommand.() -> Unit): ContinuousReuseCommand {
    return ContinuousReuseCommand().apply(inner)
}

@MoonshineDsl
fun subsystems(inner: ContinuousReuseCommand.() -> Unit): ContinuousReuseCommand {
    return ContinuousReuseCommand().apply(inner)
}

@MoonshineDsl
fun<T: Subsystem> ContinuousReuseCommand.add(subsystem: SharedSubsystem<T>) {
    this.children += subsystem.value
}

//
//@MoonshineDsl
//inline fun<reified T: Subsystem> subsystemOfType(defaultValue: T): SharedSubsystem<T> {
//    val subsystem = SharedSubsystem(T::class.java)
//    subsystem.value = defaultValue
//    return subsystem
//}
