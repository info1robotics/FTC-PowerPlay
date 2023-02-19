package org.firstinspires.ftc.teamcode.moonshine.dsl

import org.firstinspires.ftc.teamcode.moonshine.Command
import org.firstinspires.ftc.teamcode.moonshine.builtin.ParallelCommand
import org.firstinspires.ftc.teamcode.moonshine.builtin.PassthroughCommand
import org.firstinspires.ftc.teamcode.moonshine.builtin.SerialCommand
import org.firstinspires.ftc.teamcode.moonshine.builtin.SleepCommand
import org.firstinspires.ftc.teamcode.moonshine.builtin.SwitchCommand
import java.util.function.IntSupplier
import java.util.function.Supplier

@DslMarker
annotation class MoonshineDsl

@MoonshineDsl
operator fun Command.unaryPlus() {
    this.children += this@unaryPlus
}

@MoonshineDsl
fun Command.serial(inner: SerialCommand.() -> Unit) /*where T : @SupportsMultipleChildren Command*/ {
    this.children += SerialCommand().apply(inner)
}

@MoonshineDsl
fun Command.parallel(shorting: Boolean, inner: ParallelCommand.() -> Unit) {
    this.children += ParallelCommand(shorting).apply(inner)
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
fun commandRoot(inner: PassthroughCommand.() -> Unit): PassthroughCommand {
    return PassthroughCommand().apply(inner)
}