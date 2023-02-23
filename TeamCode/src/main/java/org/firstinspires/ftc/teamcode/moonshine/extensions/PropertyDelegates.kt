package org.firstinspires.ftc.teamcode.moonshine.extensions

import org.firstinspires.ftc.teamcode.moonshine.CommandEnv
import org.firstinspires.ftc.teamcode.moonshine.SharedSubsystem
import org.firstinspires.ftc.teamcode.moonshine.SharedVar
import org.firstinspires.ftc.teamcode.moonshine.Subsystem
import kotlin.reflect.KProperty

class InjectHardware<T>(private val hardwareName: String? = null) {
    operator fun getValue(thisRef: Any?, property: KProperty<*>): T {
        return CommandEnv.getInstance().eventLoop.opModeManager.hardwareMap[hardwareName ?: property.name] as T
    }

}

open class InjectSharedRoot<T, E: SharedVar<T>>(
    private val defaultRootProvider: (KProperty<*>) -> E,
    private val defaultValue: T? = null) {

    lateinit var sharedRoot: E

    private fun initIfNeeded(property: KProperty<*>) {
        if(!::sharedRoot.isInitialized) {
            sharedRoot = defaultRootProvider(property)
        }
    }

    operator fun getValue(thisRef: Any?, property: KProperty<*>): T {
        initIfNeeded(property)
        return sharedRoot.value
    }

    operator fun setValue(thisRef: Any?, property: KProperty<*>, value: T) {
        initIfNeeded(property)
        sharedRoot.value = value
    }

}

class InjectSharedVar<T>(defaultValue: T? = null) :
    InjectSharedRoot<T, SharedVar<T>>({ prop ->
        SharedVar<T>(
            prop.name,
            defaultValue
        )
    }, defaultValue)

//class InjectSubsystem<T: Subsystem>(defaultValue: T? = null, classType: T? = null) :
//    InjectSharedRoot<T, SharedSubsystem<T>>({ prop ->
//        SharedSubsystem(defaultValue.javaClass ?: classType, defaultValue)
//    }, defaultValue)
