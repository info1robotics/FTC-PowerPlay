package org.firstinspires.ftc.teamcode.moonshine.extensions

import org.firstinspires.ftc.teamcode.moonshine.Command
import org.firstinspires.ftc.teamcode.moonshine.SharedSubsystem
import org.firstinspires.ftc.teamcode.moonshine.Subsystem
import org.firstinspires.ftc.teamcode.moonshine.builtin.InlineCommand
import org.firstinspires.ftc.teamcode.moonshine.builtin.ParallelCommand
import org.firstinspires.ftc.teamcode.moonshine.builtin.PassthroughCommand
import org.firstinspires.ftc.teamcode.moonshine.builtin.SerialCommand
import org.firstinspires.ftc.teamcode.moonshine.builtin.SleepCommand
import org.firstinspires.ftc.teamcode.moonshine.builtin.SubsystemsGroup
import org.firstinspires.ftc.teamcode.moonshine.builtin.SwitchCommand
import org.firstinspires.ftc.teamcode.moonshine.builtin.TriggerCommand
import java.util.function.Supplier

@DslMarker
annotation class MoonshineDsl

@MoonshineDsl
operator fun Command.unaryPlus() {
    this.childrenCommands += this@unaryPlus
}

@MoonshineDsl
fun Command.serial(inner: SerialCommand.() -> Unit) /*where T : @SupportsMultipleChildren Command*/ {
    this.childrenCommands += SerialCommand().apply(inner)
}

@MoonshineDsl
fun Command.parallel(shorting: Boolean, inner: ParallelCommand.() -> Unit) {
    this.childrenCommands += ParallelCommand(shorting).apply(inner)
}

@MoonshineDsl
fun Command.sleep(durationMs: Long) {
    this.childrenCommands += SleepCommand(durationMs)
}

@MoonshineDsl
fun Command.switch(switchResult: Supplier<Int>, inner: SwitchCommand.() -> Unit) {
    this.childrenCommands += SwitchCommand(switchResult).apply(inner)
}

@MoonshineDsl
fun Command.trigger(cond: Supplier<Boolean>, inner: TriggerCommand.() -> Unit) {
    this.childrenCommands += TriggerCommand(cond).apply(inner)
}

@MoonshineDsl
fun Command.inline(func: () -> Unit) {
    this.childrenCommands += InlineCommand(func)
}

@MoonshineDsl
fun newAutoRoutine(inner: PassthroughCommand.() -> Unit): PassthroughCommand {
    return PassthroughCommand(false).apply(inner)
}

@MoonshineDsl
fun newTeleopRoutine(inner: PassthroughCommand.() -> Unit): PassthroughCommand {
    return PassthroughCommand(true).apply(inner)
}

@MoonshineDsl
fun subsystems(inner: SubsystemsGroup.() -> Unit): SubsystemsGroup {
    return SubsystemsGroup().apply(inner)
}

@MoonshineDsl
fun<T: Subsystem> SubsystemsGroup.add(sharedSubsystem: SharedSubsystem<T>) {
    this.childrenSubsystems += sharedSubsystem.value
}

@MoonshineDsl
inline fun<reified T: Subsystem> subsystemOfType(): SharedSubsystem<T> {
    return SharedSubsystem(T::class.java)
}
