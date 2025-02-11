package com.example.sdpapp

import android.Manifest
import android.annotation.SuppressLint
import android.app.Service
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Binder
import android.os.IBinder
import android.util.Log
import androidx.core.app.ActivityCompat
import java.util.UUID

private const val TAG = "RangerBluetoothService"

class RangerBluetoothService : Service() {
    private val binder = LocalBinder()

    private val SERVICE_UUID = UUID.fromString(" // TODO ") // TODO

    private var bluetoothAdapter: BluetoothAdapter? = null

    private var bluetoothGatt: BluetoothGatt? = null

    private var connectionState = STATE_DISCONNECTED

    private val reconnectHandler: android.os.Handler = android.os.Handler()
    private val reconnectDelayMillis: Long = 5000L // 5 seconds delay

    private fun reconnect(device: BluetoothDevice) {
        reconnectHandler.postDelayed({
            Log.i(TAG, "Attempting to reconnect...")

            if (ActivityCompat.checkSelfPermission(
                    this,
                    Manifest.permission.BLUETOOTH_CONNECT
                ) != PackageManager.PERMISSION_GRANTED
            ) {
                Log.e(TAG, "Bluetooth connect permission missing.")
                return@postDelayed
            }

            bluetoothGatt?.close() // Ensure old connection is closed
            bluetoothGatt = device.connectGatt(this, false, rangerGattCallback)
        }, reconnectDelayMillis)
    }

    private val rangerGattCallback: BluetoothGattCallback = object : BluetoothGattCallback() {
        @SuppressLint("MissingPermission")
        override fun onConnectionStateChange(gatt: BluetoothGatt?, status: Int, newState: Int) {
            when (newState) {
                BluetoothProfile.STATE_CONNECTED -> {
                    connectionState = STATE_CONNECTED
                    broadcastUpdate(ACTION_GATT_CONNECTED)
                    Log.i(TAG, "Connected to GATT server")
                }
                BluetoothProfile.STATE_DISCONNECTED -> {
                    connectionState = STATE_DISCONNECTED
                    broadcastUpdate(ACTION_GATT_DISCONNECTED)
                    Log.e(TAG, "Disconnected from GATT server")

                    gatt?.device?.let { device ->
                        reconnect(device) // Use separate function
                    }
                }
            }
        }
    }



    private fun broadcastUpdate(action: String) {
        val intent = Intent(action)
        sendBroadcast(intent)
    }

    fun connect(address: String): Boolean {
        bluetoothAdapter?.let { adapter ->
            try {
                val device = adapter.getRemoteDevice(address)
                if (ActivityCompat.checkSelfPermission(
                        this,
                        Manifest.permission.BLUETOOTH_CONNECT
                    ) != PackageManager.PERMISSION_GRANTED
                ) {
                    Log.e(TAG, "No bluetooth connection permissions")
                    return false
                }
                bluetoothGatt = device.connectGatt(this, false, rangerGattCallback)
                return true
            } catch (exception: IllegalArgumentException) {
                Log.w(TAG, "Device not found with provided address.")
                return false
            }
        } ?: run {
            Log.w(TAG, "BluetoothAdapter not initialized")
            return false
        }

        throw IllegalStateException("unreachable")
    }

    fun initialize(): Boolean {
        val bm = getSystemService(Context.BLUETOOTH_SERVICE)
        if (bm is BluetoothManager) {
            bluetoothAdapter = bm.adapter
            if (bluetoothAdapter == null) {
                Log.e(TAG, "Unable to obtain a BluetoothAdapter")
                return false
            }
            return true
        } else {
            Log.e(TAG, "Unable to obtain a BluetoothManager")
            return false
        }
    }

    override fun onBind(p0: Intent?): IBinder? {
        return binder
    }

    inner class LocalBinder : Binder() {
        fun getService() : RangerBluetoothService {
            return this@RangerBluetoothService
        }
    }

    companion object {
        const val ACTION_GATT_CONNECTED =
            "com.example.sdpapp.ACTION_GATT_CONNECTED"
        const val ACTION_GATT_DISCONNECTED =
            "com.example.sdpapp.ACTION_GATT_DISCONNECTED"

        private const val STATE_DISCONNECTED = 0

        private const val STATE_CONNECTED = 2

        const val REQUEST_BLUETOOTH_CONNECT = 1001
    }

    @SuppressLint("MissingPermission")
    fun writeToCharacteristic(value: ByteArray) {
        val gatt = bluetoothGatt ?: return

        val characteristic = gatt.getService(SERVICE_UUID)?.getCharacteristic(SERVICE_UUID)

        if (characteristic != null) {
            characteristic.value = value
            gatt.writeCharacteristic(characteristic)
            Log.i(TAG, "Command sent: ${String(value)}")
        } else {
            Log.e(TAG, "Characteristic not found!")
        }
    }

}