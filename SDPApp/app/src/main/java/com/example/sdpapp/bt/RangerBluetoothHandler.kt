package com.example.sdpapp.bt

import android.annotation.SuppressLint
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothGattService
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.BluetoothStatusCodes
import android.content.Context
import android.content.Context.BLUETOOTH_SERVICE
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.widget.Toast
import androidx.annotation.RequiresApi
import com.example.sdpapp.ui.demoStartedFunction
import java.io.Closeable
import java.util.LinkedList
import java.util.Queue
import java.util.UUID
import java.util.concurrent.CompletableFuture
import java.util.concurrent.CompletionStage
import java.util.concurrent.Future
import java.util.concurrent.TimeUnit
import java.util.concurrent.TimeoutException

private const val TAG = "RangerBluetoothHandler"

private const val RANGER_SERVICE_UUID = "fbb876fb-3ee3-5315-9716-01ede2358aab"
private const val START_DEMO_UUID = "82e761bc-8508-5f80-90ee-9b3455444798"
private const val CANCEL_DEMO_UUID = "19a368f7-b27f-557b-81c5-be1130a406f5"
private const val POISON_STATE_UUID = "286d24e6-5611-51b2-a2b3-6fb9d9aa9566"
private const val RESET_POISON_UUID = "c0d915c8-26b1-50da-951e-d91bc4d3c5e1"

class RangerBluetoothHandler private constructor
    ( private val gatt: BluetoothGatt
    , private val demoStartChar: BluetoothGattCharacteristic
    , private val demoCancelChar: BluetoothGattCharacteristic
    , private val poisonStateChar: BluetoothGattCharacteristic
    , private val resetPoisonChar: BluetoothGattCharacteristic
    , private val ctx: Context
): Closeable
{
    var isConnected = true

    private var readRequests: Queue<CompletableFuture<ByteArray>> = LinkedList()

    @RequiresApi(Build.VERSION_CODES.S)
    @SuppressLint("MissingPermission")
    private fun queueRead(ch: BluetoothGattCharacteristic): CompletionStage<ByteArray> {
        Log.d(TAG, "Queueing read to $ch")
        val f = CompletableFuture<ByteArray>()
        readRequests.add(f)
        val readResult = gatt.readCharacteristic(ch)
        if (!readResult) {
            return CompletableFuture.failedFuture(RuntimeException("Read failed early"))
        }
        return f
    }

    private var writeIndex: UByte = 0u

    private fun resetWriteIndex() {
        writeIndex = 0u
    }

    private fun incWriteIndex() {
        writeIndex =
            if (writeIndex == UByte.MAX_VALUE) {
                0u
            } else {
                (writeIndex + 1u).toUByte()
            }
    }

    @SuppressLint("MissingPermission")
    override fun close() {
        isConnected = false
        gatt.close()
    }

    @RequiresApi(Build.VERSION_CODES.TIRAMISU)
    @SuppressLint("MissingPermission")
    fun startDemo(): Boolean {
        val r = gatt.writeCharacteristic(demoStartChar, byteArrayOf(writeIndex.toByte()),
            BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT)
        if (r == BluetoothStatusCodes.SUCCESS) {
            incWriteIndex()
            Log.i(TAG, "Demo started")
            demoStartedFunction()
            return true
        } else {
            Log.e(TAG, "Failed to write to demo start: $r")
            return false
        }
    }

    @RequiresApi(Build.VERSION_CODES.TIRAMISU)
    @SuppressLint("MissingPermission")
    fun cancelDemo(): Boolean {
        val r = gatt.writeCharacteristic(demoCancelChar, byteArrayOf(writeIndex.toByte()),
            BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT)
        if (r == BluetoothStatusCodes.SUCCESS) {
            incWriteIndex()
            Log.i(TAG, "Demo cancelled")
            return true
        } else {
            Log.e(TAG, "Failed to write to cancel demo: $r")
            return false
        }
    }

    @RequiresApi(Build.VERSION_CODES.TIRAMISU)
    @SuppressLint("MissingPermission")
    private fun resetPoison(): Future<Boolean> {
        val writeResult = gatt.writeCharacteristic(resetPoisonChar, ByteArray(0), BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE)
        if (writeResult != BluetoothStatusCodes.SUCCESS) {
            return CompletableFuture.completedFuture(false)
        }
        val readResult = queueRead(poisonStateChar)
        return readResult
            .handle<Boolean>{x,t -> t == null && x.getOrNull(0)?.toInt() == 0}
            .thenApply{b ->
                resetWriteIndex()
                b
            }
            .toCompletableFuture()
    }

    companion object {
        @SuppressLint("MissingPermission")
        @RequiresApi(Build.VERSION_CODES.S)
        fun connect(ctx: Context, address: String, onDisconnectCallback: Runnable): Future<RangerBluetoothHandler> {
            val bm = ctx.getSystemService(BLUETOOTH_SERVICE)
            if (bm !is BluetoothManager) {
                return CompletableFuture.failedFuture(IllegalStateException("Bluetooth service is not BluetoothManager"))
            }
            val a = bm.adapter
            val device = a.getRemoteDevice(address)

            val h: CompletableFuture<RangerBluetoothHandler> = CompletableFuture()

            device.connectGatt(ctx, false, bluetoothGattCallback(ctx, h, onDisconnectCallback))

            return h
        }

        private fun rangerService(gatt: BluetoothGatt): BluetoothGattService? {
            for (service in gatt.services) {
                val uuid = service.uuid.toString()
                Log.d(TAG, uuid)
                if (uuid != RANGER_SERVICE_UUID) {
                    continue
                }
                Log.i(TAG, "Found ranger service")
                return service
            }
            Log.e(TAG, "Did not find ranger service")
            return null
        }

        @SuppressLint("MissingPermission")
        private fun bluetoothGattCallback(
            ctx: Context,
            h: CompletableFuture<RangerBluetoothHandler>,
            onDisconnectCallback: Runnable
        ): BluetoothGattCallback = object : BluetoothGattCallback() {
            override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
                Log.d(TAG, "Connection state change")
                if (status != BluetoothGatt.GATT_SUCCESS) {
                    Handler(Looper.getMainLooper()).post {
                        Toast.makeText(
                            ctx,
                            "Unable to connect. Make sure the robot is turned on and in range.",
                            Toast.LENGTH_LONG
                        ).show()
                    }
                    Log.e(TAG, "GATT connection state change did not return success")
                }
                if (newState == BluetoothProfile.STATE_CONNECTED) {
                    Log.i(TAG, "Connected to device")

                    gatt.discoverServices()
                } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                    val h1 = h.getNow(null)
                    h1?.isConnected = false
                    gatt.close()
                    if (h1 != null) {
                        Log.i(TAG, "Disconnected from device")
                        onDisconnectCallback.run()
                    } else {
                        Log.w(TAG, "Disconnected before service discovery")
                    }
                }
            }

            override fun onCharacteristicRead(
                gatt: BluetoothGatt,
                characteristic: BluetoothGattCharacteristic,
                value: ByteArray,
                status: Int
            ) {
                val h1 = h.getNow(null)

                if (h1 == null) {
                    Log.w(TAG, "Received read response before service discovery")
                    return
                }

                val f = h1.readRequests.poll()

                if (f == null) {
                    Log.w(TAG, "Received read response without corresponding read request")
                    return
                }

                if (status == BluetoothGatt.GATT_SUCCESS) {
                    f.complete(value)
                } else {
                    f.completeExceptionally(RuntimeException("Read failed late"))
                }
            }

            @RequiresApi(Build.VERSION_CODES.TIRAMISU)
            override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
                if (status != BluetoothGatt.GATT_SUCCESS) {
                    Log.e(TAG, "Service discovery failed")
                    gatt.close()
                    h.completeExceptionally(RuntimeException("Service discovery failed"))
                    return
                }

                Log.d(TAG, "Services discovered")
                val s = rangerService(gatt)

                if (s == null) {
                    Log.e(TAG, "Device did not have required services, disconnecting")

                    gatt.close()
                    h.completeExceptionally(RuntimeException("Services not found on device"))
                    return
                }

                var demoStartChar: BluetoothGattCharacteristic? = null
                var demoCancelChar: BluetoothGattCharacteristic? = null
                var poisonStateChar: BluetoothGattCharacteristic? = null
                var resetPoisonChar: BluetoothGattCharacteristic? = null

                for (c in s.characteristics) {
                    when (c.uuid.toString()) {
                        START_DEMO_UUID -> {
                            Log.d(TAG, "Found start demo characteristic")
                            demoStartChar = c
                        }

                        CANCEL_DEMO_UUID -> {
                            Log.d(TAG, "Found start demo characteristic")
                            demoCancelChar = c
                        }

                        POISON_STATE_UUID -> {
                            Log.d(TAG, "Found poison state characteristic")
                            poisonStateChar = c
                        }

                        RESET_POISON_UUID -> {
                            Log.d(TAG, "Found reset poison characteristic")
                            resetPoisonChar = c
                        }
                    }
                }

                if (listOf(demoStartChar, demoCancelChar, poisonStateChar, resetPoisonChar).any { it == null }
                ) {
                    Log.e(
                        TAG,
                        "Service found, but does not provide all characteristics, disconnecting"
                    )
                    gatt.close()
                    h.completeExceptionally(RuntimeException("Missing characteristics on ranger service"))
                    return
                }

                Log.i(TAG, "Found all characteristics")

                Handler(Looper.getMainLooper()).post {
                    Toast.makeText(
                        ctx,
                        "Connected to Robot.",
                        Toast.LENGTH_LONG
                    ).show()
                }

                val handler = RangerBluetoothHandler(gatt, demoStartChar!!, demoCancelChar!!, poisonStateChar!!, resetPoisonChar!!, ctx)

                val res1 = gatt.setCharacteristicNotification(handler.poisonStateChar, true)
                val d = handler.poisonStateChar.getDescriptor(UUID.fromString("00002902-0000-1000-8000-00805f9b34fb"))
                val res2 = gatt.writeDescriptor(d, BluetoothGattDescriptor.ENABLE_INDICATION_VALUE)

                val resetPoisonRes = handler.resetPoison().get(3, TimeUnit.SECONDS)

                if (resetPoisonRes && res1 && res2 == BluetoothStatusCodes.SUCCESS) {
                    h.complete(handler)
                } else {
                    h.completeExceptionally(RuntimeException("Failed to enable indications on poison state"))
                }

            }

            @RequiresApi(Build.VERSION_CODES.TIRAMISU)
            override fun onCharacteristicChanged(
                gatt: BluetoothGatt,
                characteristic: BluetoothGattCharacteristic,
                value: ByteArray
            ) {
                when (characteristic.uuid.toString()) {
                    POISON_STATE_UUID -> {
                        if (value.getOrNull(0)?.toInt() == 1) {
                            val h1 = h.getNow(null)

                            if (h1 == null) {
                                Log.w(TAG, "Entered poison state before service discovery")
                                return
                            }

                            Log.w(TAG, "Poison state active! Attempting to reset...")

                            try {
                                val result = h1.resetPoison().get(3, TimeUnit.SECONDS)

                                if (result) {
                                    Log.i(TAG, "Poison state reset")
                                } else {
                                    Log.e(TAG, "Failed to reset poison state")
                                    gatt.close()
                                }
                            } catch (_: TimeoutException) {
                                Log.e(TAG, "Failed to reset poison state (timeout)")
                                gatt.close()
                            }

                        }
                    }
                }
            }
        }

    }
}