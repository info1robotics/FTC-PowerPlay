package org.firstinspires.ftc.teamcode.moonshine.dsl

import org.firstinspires.ftc.teamcode.moonshine.CommandEnv
import kotlin.properties.ReadOnlyProperty
import kotlin.reflect.KProperty

class InjectHardware<T> {
    operator fun getValue(thisRef: Any?, property: KProperty<*>): T {
        return CommandEnv.getInstance().eventLoop.opModeManager.hardwareMap[property.name] as T
    }

}

class InjectSharedVar<T> {
    operator fun getValue(thisRef: Any?, property: KProperty<*>): T {
        return CommandEnv.getInstance().sharedVars[property.name] as T
    }

    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: T) {
        CommandEnv.getInstance().sharedVars[property.name] = value
    }
}