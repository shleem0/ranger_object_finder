package com.example.sdpapp.bt

import android.Manifest
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.BluetoothStatusCodes
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Binder
import android.os.Build
import android.os.Handler
import android.os.IBinder
import android.os.Looper
import android.util.Log
import android.widget.Toast
import androidx.annotation.RequiresApi
import androidx.compose.runtime.Composable
import androidx.compose.ui.platform.LocalContext
import androidx.core.app.ActivityCompat
import androidx.core.app.NotificationCompat
import com.example.sdpapp.R
import java.lang.IllegalStateException

/**
* placeholder, **should replace this with the Raspberry Pi's address**
*
* eventually, could make it automatically find the device by the available service UUIDs:
*
* see example https://github.com/bluez/bluer/blob/master/bluer/examples/gatt_client.rs
*/
private const val RANGER_DEMO_ADDRESS = "90:32:4B:8F:FC:D6"

private const val TAG = "RangerBluetoothService"

private const val DEMO_SERVICE_UUID = "fbb887fb-3ee3-5315-9716-01ede2358aab"
private const val IS_DEMO_ACTIVE_UUID = "82e761bc-8508-5f80-90ee-9b3455444798"


class RangerBluetoothService : Service() {

    fun getConnectionState(): Int {
        return connectionState
    }

    private var connectionState = STATE_DISCONNECTED

    private var bluetoothAdapter: BluetoothAdapter? = null

    private var bluetoothGatt: BluetoothGatt? = null

    private var isDemoActiveCharacteristic: BluetoothGattCharacteristic? = null

    private val binder = LocalBinder()

    private fun tryGetBluetoothAdapter(): BluetoothAdapter? {
        val bm = getSystemService(BLUETOOTH_SERVICE)
        if (bm is BluetoothManager) {
            val a = bm.adapter
            return a
        } else {
            return null
        }
    }

    public fun createNotificationChannel() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            val name = "Search Progress"
            val descriptionText = "Notifications for Ranger Search"
            val importance = NotificationManager.IMPORTANCE_HIGH
            val channel = NotificationChannel(CHANNEL_ID, name, importance).apply {
                description = descriptionText
            }
            val notificationManager: NotificationManager =
                getSystemService(NotificationManager::class.java)
            notificationManager.createNotificationChannel(channel)
        }
    }

    private fun showProgressNotification() {
        val notificationManager = getSystemService(NotificationManager::class.java)
        val builder = NotificationCompat.Builder(this, CHANNEL_ID)
            .setSmallIcon(R.mipmap.appicon)
            .setContentTitle("Ranger Search")
            .setContentText("Searching for target...")
            .setProgress(0, 0, true)
            .setOngoing(true)
            .setPriority(NotificationCompat.PRIORITY_HIGH)

        notificationManager.notify(NOTIFICATION_ID, builder.build())
    }

    private fun showFoundNotification() {
        val notificationManager = getSystemService(NotificationManager::class.java)
        val builder = NotificationCompat.Builder(this, CHANNEL_ID)
            .setSmallIcon(R.mipmap.appicon)
            .setContentTitle("Ranger Alert!")
            .setContentText("Target found!")
            .setPriority(NotificationCompat.PRIORITY_HIGH)
            .setAutoCancel(true)

        notificationManager.notify(NOTIFICATION_ID, builder.build())
    }

    private fun cancelProgressNotification() {
        val notificationManager = getSystemService(NotificationManager::class.java)
        notificationManager.cancel(NOTIFICATION_ID)
    }

    override fun onBind(p0: Intent?): IBinder {
        Log.d(TAG, "binding service2")
        return binder
    }

    /**
     * Should be run before using the service.
     *
     * Returns true if initialisation was successful, false otherwise.
     */
    fun initialize(): Boolean {
        val a = tryGetBluetoothAdapter()
        if (a == null) {
            Log.e(TAG, "Failed to get bluetooth adapter")
            return false
        }
        bluetoothAdapter = a
        return true
    }

    /**
     * 'connect' specialised to the fixed demo address
     */

    fun connectForDemo(): Boolean {
        return this.connect(RANGER_DEMO_ADDRESS)
    }

    private fun broadcastUpdate(action: String) {
        val intent = Intent(action)
        sendBroadcast(intent)

        if (action == ACTION_GATT_READY) {
            showFoundNotification()
            cancelProgressNotification()
        }
    }

    private val bluetoothGattCallback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(gatt: BluetoothGatt?, status: Int, newState: Int) {
            Log.d(TAG, "Connection state change")
            if (status != BluetoothGatt.GATT_SUCCESS) {
                Handler(Looper.getMainLooper()).post {
                    Toast.makeText(
                        this@RangerBluetoothService,
                        "Unable to connect. Make sure the robot is turned on and in range",
                        Toast.LENGTH_LONG
                    ).show()
                }
                Log.e(TAG, "GATT connection state change did not return success")
            }
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                connectionState = STATE_CONNECTED
                Log.i(TAG, "connected")
                broadcastUpdate(ACTION_GATT_CONNECTED)

                // More permission check boilerplate
                if (ActivityCompat.checkSelfPermission(
                        this@RangerBluetoothService,
                        Manifest.permission.BLUETOOTH_CONNECT
                    ) != PackageManager.PERMISSION_GRANTED
                ) {
                    return
                }

                bluetoothGatt?.discoverServices()
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                connectionState = STATE_DISCONNECTED
                broadcastUpdate(ACTION_GATT_DISCONNECTED)
            }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt?, status: Int) {
            Log.d(TAG, "Services discovered")
            if (status == BluetoothGatt.GATT_SUCCESS) {
                gatt ?: throw IllegalStateException("gatt is null, but we just connected")

                val ch = is_demo_active_char(gatt)

                if (ch == null) {
                    Log.e(TAG, "Connected to device, but device did not have required services, disconnecting")
                    close()
                    return
                }

                isDemoActiveCharacteristic = ch

                connectionState = STATE_READY
                broadcastUpdate(ACTION_GATT_READY)
            } else if (status == BluetoothGatt.GATT_FAILURE) {
                Log.e(TAG, "service discovery failed, disconnecting: $status")
                close()
            } else {
                Log.w(TAG, "BluetoothGatt returned: $status")
            }
        }
    }

    @RequiresApi(Build.VERSION_CODES.TIRAMISU)
    fun startDemo(): Boolean {
        val gatt = bluetoothGatt
        if (gatt == null) {
            Log.e(TAG, "Can't start demo, no connection")
            return false
        }
        val ch = isDemoActiveCharacteristic
        if (ch == null) {
            Log.e(TAG, "Don't have demo characteristic yet, can't write to it")
            return false
        }

        // more permission check boilerplate
        if (ActivityCompat.checkSelfPermission(
                this,
                Manifest.permission.BLUETOOTH_CONNECT
            ) != PackageManager.PERMISSION_GRANTED
        ) {
            Log.e(TAG, "Can't start demo, lost BLUETOOTH_CONNECT permissions")
            return false
        }
        val r = gatt.writeCharacteristic(ch, byteArrayOf(1), BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT)

        if (r == BluetoothStatusCodes.SUCCESS) {
            Log.i(TAG, "Demo started")
            showProgressNotification()
            return true
        } else {
            Log.e(TAG, "Failed to write characteristic: writeCharacteristic returned $r")
            return false
        }
    }

    /**
     * Gets the characteristic in DEMO_SERVICE_UUID -> IS_DEMO_ACTIVE_UUID
     */
    private fun is_demo_active_char(gatt: BluetoothGatt): BluetoothGattCharacteristic? {
        var is_demo_active_char: BluetoothGattCharacteristic? = null

        for (service in gatt.services) {
            val uuid = service.uuid.toString()
            if (uuid != DEMO_SERVICE_UUID) {
                continue
            }
            Log.i(TAG, "Found service with required UUID")
            val cs = service.characteristics
            for (c in cs) {
                if (c.uuid.toString() == IS_DEMO_ACTIVE_UUID) {
                    Log.i(TAG, "Found characteristic with required UUID")
                    is_demo_active_char = c
                    break
                }
            }
            // service with required uuid was found,
            break
        }

        if (is_demo_active_char != null) {
            Log.i(TAG, "Found demo service")
        } else {
            Log.e(TAG, "Did not find demo service")
        }

        return is_demo_active_char
    }

    private fun close() {
        bluetoothGatt?.let { gatt ->

            bluetoothGatt = null
            isDemoActiveCharacteristic = null
            connectionState = STATE_DISCONNECTED

            if (ActivityCompat.checkSelfPermission(
                    this,
                    Manifest.permission.BLUETOOTH_CONNECT
                ) != PackageManager.PERMISSION_GRANTED
            ) {
                Log.e(TAG, "No BLUETOOTH_CONNECT permission, won't close GATT connection")
                return
            }
            gatt.close()
        }
    }

    /**
     * _Begin_ connecting to the given Bluetooth MAC address. Returns true if the device exists
     * and we have permissions to connect to it.
     *
     * Once we are actually connected, the service will broadcast ACTION_GATT_CONNECTED,
     * then ACTION_GATT_READY once we know the services.
     */
    fun connect(address: String): Boolean {
        Log.d(TAG, "Connect request")
        bluetoothAdapter?.let { adapter ->
            try {
                val device = adapter.getRemoteDevice(address)

                // Permission check boilerplate
                // pani, TODO: app-wide PermissionManager class for this that shows the user a prompt
                if (ActivityCompat.checkSelfPermission(
                        this,
                        Manifest.permission.BLUETOOTH_CONNECT
                    ) != PackageManager.PERMISSION_GRANTED
                ) {
                    Log.e(TAG, "No bluetooth connection permission, can't connect")
                    return false
                }

                val bg = device.connectGatt(this, false, bluetoothGattCallback)
                val connected = bg.connect()
                Log.d(TAG, "Connection status: $connected")
                bluetoothGatt = bg
                return connected
            } catch (exception: IllegalArgumentException) {
                Log.e(TAG, "Device not found with the provided address $address")
                return false
            }
        }

        Log.e(TAG, "(!!!) RangerBluetoothService is not initialized!")
        return false
    }

    inner class LocalBinder : Binder() {
        fun getService() : RangerBluetoothService {
            return this@RangerBluetoothService
        }
    }

    companion object {
        const val ACTION_GATT_CONNECTED =
            "com.example.sdpapp.bt.ACTION_GATT_CONNECTED"
        const val ACTION_GATT_DISCONNECTED =
            "com.example.sdpapp.bt.ACTION_GATT_DISCONNECTED"
        const val ACTION_GATT_READY =
            "com.example.sdpapp.bt.ACTION_GATT_READY"

        const val STATE_DISCONNECTED = 0
        const val STATE_CONNECTED = 2
        const val STATE_READY = 3

        const val CHANNEL_ID = "ranger_search_channel"
        const val NOTIFICATION_ID = 1
    }
}