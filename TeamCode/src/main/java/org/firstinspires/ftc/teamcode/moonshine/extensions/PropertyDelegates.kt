package org.firstinspires.ftc.teamcode.moonshine.extensions

import org.firstinspires.ftc.teamcode.moonshine.CommandEnv
import org.firstinspires.ftc.teamcode.moonshine.SharedVar
import kotlin.reflect.KProperty

class InjectHardware<T> {
    operator fun getValue(thisRef: Any?, property: KProperty<*>): T {
        return CommandEnv.getInstance().eventLoop.opModeManager.hardwareMap[property.name] as T
    }

}

class InjectSharedVar<T> {
    lateinit var sharedVar: SharedVar<T>

    fun initIfNeeded(property: KProperty<*>) {
        if(!::sharedVar.isInitialized)
            sharedVar = SharedVar<T>(property.name)
    }

    operator fun getValue(thisRef: Any?, property: KProperty<*>): T {
        initIfNeeded(property)

        return CommandEnv.getInstance().sharedVars[property.name] as T
    }

    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: T) {
        initIfNeeded(property)

        if(!::sharedVar.isInitialized)
            sharedVar = SharedVar<T>(property.name)
        CommandEnv.getInstance().sharedVars[property.name] = value
    }
}