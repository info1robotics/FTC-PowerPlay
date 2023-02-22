package org.firstinspires.ftc.teamcode.moonshine.extensions

import org.firstinspires.ftc.teamcode.moonshine.CommandEnv
import org.firstinspires.ftc.teamcode.moonshine.SharedSubsystem
import org.firstinspires.ftc.teamcode.moonshine.SharedVar
import org.firstinspires.ftc.teamcode.moonshine.Subsystem
import kotlin.reflect.KProperty

class InjectHardware<T> {
    operator fun getValue(thisRef: Any?, property: KProperty<*>): T {
        return CommandEnv.getInstance().eventLoop.opModeManager.hardwareMap[property.name] as T
    }

}

open class InjectSharedRoot<T, E: SharedVar<T>>(
    private val defaultRootProvider: (KProperty<*>) -> E,
    private val defaultValue: T) {

    lateinit var sharedRoot: E

    private fun initIfNeeded(property: KProperty<*>) {
        if(!::sharedRoot.isInitialized) {
            sharedRoot = defaultRootProvider(property)
            sharedRoot.value = defaultValue
        }
    }

    operator fun getValue(thisRef: Any?, property: KProperty<*>): T {
        initIfNeeded(property)
        return CommandEnv.getInstance().sharedVars[property.name] as T
    }

    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: T) {
        initIfNeeded(property)
        CommandEnv.getInstance().sharedVars[property.name] = value
    }

}

class InjectSharedVar<T>(defaultValue: T) :
    InjectSharedRoot<T, SharedVar<T>>({ prop ->
        SharedVar<T>(
            prop.name
        )
    }, defaultValue)

class InjectSubsystem<T: Subsystem>(defaultValue: T) :
    InjectSharedRoot<T, SharedSubsystem<T>>({ prop ->
        SharedSubsystem(
            defaultValue.javaClass
        )
    }, defaultValue)