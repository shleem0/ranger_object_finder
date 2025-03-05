package sdpapp.bt

import android.annotation.SuppressLint
import android.app.Service
import android.bluetooth.BluetoothGatt
import android.content.Intent
import android.os.Binder
import android.os.IBinder
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.first

// Bluetooth Service Interface
interface IRangerBluetoothService {
    suspend fun initialize(): Boolean
    suspend fun connect(address: String): Boolean
    suspend fun disconnect()
    fun isConnected(): Flow<Boolean> // Connection status as Flow
    suspend fun sendItemRequest(item: String): Boolean
    suspend fun receivePhoto(): Flow<ByteArray?>
}

// Callback Interface replaced with Flow
interface IBluetoothConnectionManager {
    suspend fun connect(address: String): Boolean
    suspend fun disconnect()
    fun isConnected(): Flow<Boolean>
}

interface IDataTransferManager {
    suspend fun sendItem(item: String): Boolean
    fun receivePhoto(): Flow<ByteArray?>
}

// Actual Service Implementation
class RangerBluetoothService : Service(), IRangerBluetoothService {

    private val connectionManager: IBluetoothConnectionManager = BluetoothConnectionManager()
    private val dataTransferManager: IDataTransferManager = BluetoothDataTransferManager()

    override suspend fun initialize(): Boolean {
        return connectionManager.isConnected().first()
    }

    override suspend fun connect(address: String): Boolean {
        return connectionManager.connect(address)
    }

    override suspend fun disconnect() {
        connectionManager.disconnect()
    }

    override fun isConnected(): Flow<Boolean> {
        return connectionManager.isConnected()
    }

    override suspend fun sendItemRequest(item: String): Boolean {
        return dataTransferManager.sendItem(item)
    }

    override suspend fun receivePhoto(): Flow<ByteArray?> {
        return dataTransferManager.receivePhoto()
    }

    inner class LocalBinder : Binder() {
        fun getService(): RangerBluetoothService = this@RangerBluetoothService
    }

    override fun onBind(intent: Intent?): IBinder = LocalBinder()
}

// Connection Manager using Flow
class BluetoothConnectionManager : IBluetoothConnectionManager {
    private var bluetoothGatt: BluetoothGatt? = null
    private val _isConnected = MutableStateFlow(false)

    override suspend fun connect(address: String): Boolean {
        // Simulating connection delay
        kotlinx.coroutines.delay(2000)
        _isConnected.value = true
        return true
    }

    @SuppressLint("MissingPermission")
    override suspend fun disconnect() {
        bluetoothGatt?.close()
        bluetoothGatt = null
        _isConnected.value = false
    }

    override fun isConnected(): Flow<Boolean> = _isConnected
}

// Data Transfer Manager using Flow
class BluetoothDataTransferManager : IDataTransferManager {
    private val _photoFlow = MutableStateFlow<ByteArray?>(null)

    override suspend fun sendItem(item: String): Boolean {
        // Simulating data send delay
        kotlinx.coroutines.delay(1000)
        return true
    }

    override fun receivePhoto(): Flow<ByteArray?> {
        return _photoFlow
    }

    // Simulating receiving a photo
    suspend fun simulatePhotoReceived(photo: ByteArray) {
        _photoFlow.value = photo
    }
}