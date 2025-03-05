package sdpapp.bt

import android.annotation.SuppressLint
import android.app.Service
import android.bluetooth.BluetoothGatt
import android.content.Intent
import android.os.Binder
import android.os.IBinder

interface IRangerBluetoothService {
    fun initialize(): Boolean
    fun connect(address: String): Boolean
    fun disconnect()
    fun isConnected(): Boolean
    fun sendItemRequest(item: String): Boolean
    fun receivePhoto(): ByteArray?
    fun registerCallback(callback: RangerBluetoothCallback)
    fun unregisterCallback(callback: RangerBluetoothCallback)
}

interface RangerBluetoothCallback {
    fun onConnected()
    fun onDisconnected()
    fun onSearchStarted()
    fun onSearchCompleted(photo: ByteArray)
    fun onError(errorMessage: String)
}

interface IBluetoothConnectionManager {
    fun connect(address: String): Boolean
    fun disconnect()
    fun isConnected(): Boolean
    fun maintainConnection()
}

interface IDataTransferManager {
    fun sendItem(item: String): Boolean
    fun receivePhoto(): ByteArray?
}

class RangerBluetoothService1 : Service(), IRangerBluetoothService {

    private val connectionManager: IBluetoothConnectionManager = BluetoothConnectionManager()
    private val dataTransferManager: IDataTransferManager = BluetoothDataTransferManager()
    private val callbacks = mutableListOf<RangerBluetoothCallback>()

    override fun initialize(): Boolean {
        return connectionManager.isConnected()
    }

    override fun connect(address: String): Boolean {
        val result = connectionManager.connect(address)
        if (result) {
            callbacks.forEach { it.onConnected() }
        }
        return result
    }

    override fun disconnect() {
        connectionManager.disconnect()
        callbacks.forEach { it.onDisconnected() }
    }

    override fun isConnected(): Boolean {
        return connectionManager.isConnected()
    }

    override fun sendItemRequest(item: String): Boolean {
        return dataTransferManager.sendItem(item)
    }

    override fun receivePhoto(): ByteArray? {
        return dataTransferManager.receivePhoto()
    }

    override fun registerCallback(callback: RangerBluetoothCallback) {
        callbacks.add(callback)
    }

    override fun unregisterCallback(callback: RangerBluetoothCallback) {
        callbacks.remove(callback)
    }

    inner class LocalBinder : Binder() {
        fun getService(): RangerBluetoothService1 = this@RangerBluetoothService1
    }

    override fun onBind(intent: Intent?): IBinder? {
        TODO("")
    }
}

class BluetoothConnectionManager : IBluetoothConnectionManager {
    private var bluetoothGatt: BluetoothGatt? = null

    override fun connect(address: String): Boolean {
        // TODO
        return true
    }

    @SuppressLint("MissingPermission")
    override fun disconnect() {
        bluetoothGatt?.close()
        bluetoothGatt = null
    }

    override fun isConnected(): Boolean {
        return bluetoothGatt != null
    }

    override fun maintainConnection() {
        // TODO
    }
}

class BluetoothDataTransferManager : IDataTransferManager {
    override fun sendItem(item: String): Boolean {
        // TODO
        return true
    }

    override fun receivePhoto(): ByteArray? {
        // TODO
        return null
    }
}
