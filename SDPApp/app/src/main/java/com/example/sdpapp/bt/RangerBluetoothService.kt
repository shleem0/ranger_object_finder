package com.example.sdpapp.bt

import android.Manifest
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattService
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
private val RANGER_DEMO_ADDRESS = "90:32:4B:8F:FC:D6"
// private val RANGER_DEMO_ADDRESS = "B8:27:EB:02:F7:BB"

private const val TAG = "RangerBluetoothService"

private const val DEMO_SERVICE_UUID = "fbb876fb-3ee3-5315-9716-01ede2358aab"
private const val START_DEMO_UUID = "82e761bc-8508-5f80-90ee-9b3455444798"
private const val CANCEL_DEMO_UUID = "19a368f7-b27f-557b-81c5-be1130a406f5"
private const val POISON_STATE_UUID = "foo"
private const val RESET_POISON_UUID = "bar"


class RangerBluetoothService : Service() {

    private var writeIndex = UByte.MIN_VALUE

    private var connectionStateListener: ((Int) -> Unit)? = null

    fun getConnectionState(): Int {
        return connectionState
    }

    fun setConnectionStateListener(listener: (Int) -> Unit) {
        connectionStateListener = listener
    }

    private fun updateConnectionState(newState: Int) {
        connectionState = newState
        connectionStateListener?.invoke(newState) // Notify UI of state change
    }

    private var connectionState = STATE_DISCONNECTED

    private var bluetoothAdapter: BluetoothAdapter? = null

    private var bluetoothGatt: BluetoothGatt? = null

    private var demoStartChar: BluetoothGattCharacteristic? = null

    private var demoCancelChar: BluetoothGattCharacteristic? = null

    private var poisonStateUuid: BluetoothGattCharacteristic? = null

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

//    public fun createNotificationChannel() {
//        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
//            val name = "Search Progress"
//            val descriptionText = "Notifications for Ranger Search"
//            val importance = NotificationManager.IMPORTANCE_HIGH
//            val channel = NotificationChannel(CHANNEL_ID, name, importance).apply {
//                description = descriptionText
//            }
//            val notificationManager: NotificationManager =
//                getSystemService(NotificationManager::class.java)
//            notificationManager.createNotificationChannel(channel)
//        }
//    }
//
//    private fun showProgressNotification() {
//        val notificationManager = getSystemService(NotificationManager::class.java)
//        val builder = NotificationCompat.Builder(this, CHANNEL_ID)
//            .setSmallIcon(R.mipmap.appicon)
//            .setContentTitle("Ranger Search")
//            .setContentText("Searching for target...")
//            .setProgress(0, 0, true)
//            .setOngoing(true)
//            .setPriority(NotificationCompat.PRIORITY_HIGH)
//
//        notificationManager.notify(NOTIFICATION_ID, builder.build())
//    }
//
//    private fun showFoundNotification() {
//        val notificationManager = getSystemService(NotificationManager::class.java)
//        val builder = NotificationCompat.Builder(this, CHANNEL_ID)
//            .setSmallIcon(R.mipmap.appicon)
//            .setContentTitle("Ranger Alert!")
//            .setContentText("Target found!")
//            .setPriority(NotificationCompat.PRIORITY_HIGH)
//            .setAutoCancel(true)
//
//        notificationManager.notify(NOTIFICATION_ID, builder.build())
//    }

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
            //showFoundNotification()
            //cancelProgressNotification()
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

                val s = demo_service(gatt)

                if (s == null) {
                    Log.e(TAG, "Connected to device, but device did not have required services, disconnecting")
                    close()
                    return
                }

                val cs = s.characteristics
                for (c in cs) {
                   if (c.uuid.toString() == START_DEMO_UUID) {
                       Log.i(TAG, "Found demo start characteristic")
                       demoStartChar = c
                   }
                   if (c.uuid.toString() == CANCEL_DEMO_UUID) {
                       Log.i(TAG, "Found demo cancel characteristic")
                       demoCancelChar = c
                   }
                }

                if (demoStartChar == null || demoCancelChar == null) {
                    Log.e(TAG, "Found demo service, but service did not have expected characteristics, disconnecting")
                    close()
                    return
                }

                Log.d(TAG, "cancel characteristic is " + demoCancelChar.toString())

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
    fun startDemo(item: String): Boolean {
        val gatt = bluetoothGatt
        if (gatt == null) {
            Log.e(TAG, "Can't start Ranger, no connection")
            return false
        }
        val ch = demoStartChar
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
        val r = gatt.writeCharacteristic(ch, byteArrayOf(writeIndex.toByte()) + item.encodeToByteArray(), BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT)
        incWriteIndex()

        if (r == BluetoothStatusCodes.SUCCESS) {
            Log.i(TAG, "Demo started")
            //showProgressNotification()
            return true
        } else {
            Log.e(TAG, "Failed to write characteristic: writeCharacteristic returned $r")
            return false
        }

    }

    private fun incWriteIndex() {
        if (writeIndex == UByte.MAX_VALUE) {
            writeIndex = UByte.MIN_VALUE
        } else {
            writeIndex = (writeIndex.toByte() + 1).toUByte()
        }
    }

    /**
     * Gets the characteristic in DEMO_SERVICE_UUID -> IS_DEMO_ACTIVE_UUID
     */
    private fun demo_service(gatt: BluetoothGatt): BluetoothGattService? {

        for (service in gatt.services) {
            val uuid = service.uuid.toString()
            Log.d(TAG, uuid)
            if (uuid != DEMO_SERVICE_UUID) {
                continue
            }
            Log.i(TAG, "Found service with required UUID")

            // service with required uuid was found,
            Log.i(TAG, "Found demo service")
            return service
        }
        Log.e(TAG, "Did not find demo service")
        return null
    }
    @RequiresApi(Build.VERSION_CODES.TIRAMISU)
    fun cancelDemo(): Boolean {
        val gatt = bluetoothGatt
        if (gatt == null) {
            Log.e(TAG, "Can't start Ranger, no connection")
            return false
        }
        val ch = demoCancelChar
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
            Log.e(TAG, "Can't cancel demo, lost BLUETOOTH_CONNECT permissions")
            return false
        }

        Log.d(TAG, "Attempting to write characteristic to cancel demo")
        val r = gatt.writeCharacteristic(ch, byteArrayOf(writeIndex.toByte()) + "item".encodeToByteArray(), BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE)
        incWriteIndex()

        return if (r == BluetoothStatusCodes.SUCCESS) {
            Log.i(TAG, "Demo cancelled!!!")
            //cancelProgressNotification()
            Log.d(TAG, "returning true huhghe")
            true
        } else {
            Log.e(TAG, "Failed to write characteristic: writeCharacteristic returned $r")
            false
        }

    }
    @RequiresApi(Build.VERSION_CODES.TIRAMISU)
    fun cancelDemo1(): Boolean {
        val gatt = bluetoothGatt
        if (gatt == null) {
            Log.e(TAG, "Can't cancel Ranger demo, no connection")
            return false
        }
        val ch = demoCancelChar
        if (ch == null) {
            Log.e(TAG, "Don't have demo characteristic yet, can't write to it")
            return false
        }

        if (ActivityCompat.checkSelfPermission(
                this,
                Manifest.permission.BLUETOOTH_CONNECT
            ) != PackageManager.PERMISSION_GRANTED
        ) {
            Log.e(TAG, "Can't cancel demo, lost BLUETOOTH_CONNECT permissions")
            return false
        }

        Log.d(TAG, "writing to cancel:" + byteArrayOf(writeIndex.toByte()).contentToString())
        val r = gatt.writeCharacteristic(ch, byteArrayOf(writeIndex.toByte()), BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT)
        incWriteIndex()

        return if (r == BluetoothStatusCodes.SUCCESS) {
            Log.i(TAG, "Demo cancelled!!!")
            //cancelProgressNotification()
            true
        } else {
            Log.e(TAG, "Failed to write characteristic: writeCharacteristic returned $r")
            false
        }
    }

    fun close() {
        bluetoothGatt?.let { gatt ->

            bluetoothGatt = null
            demoStartChar = null
            demoCancelChar = null
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
//                 pani, TODO: app-wide PermissionManager class for this that shows the user a prompt
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