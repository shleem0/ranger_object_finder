package com.example.sdpapp.bt

import android.app.Service
import android.content.Intent
import android.os.Binder
import android.os.Build
import android.os.Handler
import android.os.IBinder
import android.os.Looper
import android.util.Log
import android.widget.Toast
import androidx.annotation.RequiresApi
import java.util.concurrent.ExecutionException

/**
* placeholder, **should replace this with the Raspberry Pi's address**
*
* eventually, could make it automatically find the device by the available service UUIDs:
*
* see example https://github.com/bluez/bluer/blob/master/bluer/examples/gatt_client.rs
*/
// private val RANGER_DEMO_ADDRESS = "90:32:4B:8F:FC:D6"
// private val RANGER_DEMO_ADDRESS = "B8:27:EB:02:F7:BB"
private val RANGER_DEMO_ADDRESS = "43:45:C0:00:1F:AC"

private const val TAG = "RangerBluetoothService"

class RangerBluetoothService : Service() {

    private var handler: RangerBluetoothHandler? = null

    private var connectionStateListener: ((Int) -> Unit)? = null

    private var connectionState = STATE_DISCONNECTED

    private val binder = LocalBinder()

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

    override fun onBind(p0: Intent?): IBinder {
        Log.d(TAG, "binding service2")
        return binder
    }

    /**
     * 'connect' specialised to the fixed demo address
     */
    @RequiresApi(Build.VERSION_CODES.S)
    fun connectForDemo(): Boolean {
        return connect(RANGER_DEMO_ADDRESS)
    }

    private fun broadcastUpdate(action: String) {
        val intent = Intent(action)
        sendBroadcast(intent)
    }

    @RequiresApi(Build.VERSION_CODES.TIRAMISU)
    fun startDemo(item: String): Boolean {
        val r = handler?.startDemo()
        return (r != null && r)
    }

    @RequiresApi(Build.VERSION_CODES.TIRAMISU)
    fun cancelDemo(): Boolean {
        val r = handler?.cancelDemo()
        return (r != null && r)
    }

    fun close() {
        handler?.close()
    }

    /**
     * Connect to the given Bluetooth MAC address. Returns true if we connect successfully.
     *
     * Will broadcast ACTION_GATT_READY and ACTION_GATT_DISCONNECTED at appropriate times.
     */
    @RequiresApi(Build.VERSION_CODES.S)
    fun connect(address: String): Boolean {
        val h = RangerBluetoothHandler.connect(this, address, object : Runnable {
            override fun run() {
                Log.d(TAG, "Broadcasting disconnect")
                updateConnectionState(STATE_DISCONNECTED)
                broadcastUpdate(ACTION_GATT_DISCONNECTED)
            }
        })
        try {
            handler = h.get()
            Log.d(TAG, "Broadcasting connect")
            updateConnectionState(STATE_READY)
            broadcastUpdate(ACTION_GATT_READY)

            Handler(Looper.getMainLooper()).post {
                Toast.makeText(
                    this,
                    "Connected to Robot.",
                    Toast.LENGTH_LONG
                ).show()
            }

            return true
        } catch (e: ExecutionException) {
            Log.e(TAG, "Failed to connect: $e")

            Handler(Looper.getMainLooper()).post {
                Toast.makeText(
                    this,
                    "Unable to connect. Make sure the robot is turned on and in range.",
                    Toast.LENGTH_LONG
                ).show()
            }

            return false
        }
    }

    inner class LocalBinder : Binder() {
        fun getService() : RangerBluetoothService {
            return this@RangerBluetoothService
        }
    }

    companion object {
        const val ACTION_GATT_DISCONNECTED =
            "com.example.sdpapp.bt.ACTION_GATT_DISCONNECTED"
        const val ACTION_GATT_READY =
            "com.example.sdpapp.bt.ACTION_GATT_READY"

        const val STATE_DISCONNECTED = 0
        const val STATE_READY = 3
    }
}
