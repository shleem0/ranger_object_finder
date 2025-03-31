package com.example.sdpapp.bt

import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattService
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.content.Context
import android.content.Context.BLUETOOTH_SERVICE
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.widget.Toast
import java.io.Closeable
import java.util.concurrent.CompletableFuture
import java.util.concurrent.Future

private const val TAG = "RangerBluetoothHandler"

private const val RANGER_SERVICE_UUID = "fbb876fb-3ee3-5315-9716-01ede2358aab"
private const val START_DEMO_UUID = "82e761bc-8508-5f80-90ee-9b3455444798"
private const val CANCEL_DEMO_UUID = "19a368f7-b27f-557b-81c5-be1130a406f5"
private const val POISON_STATE_UUID = "286d24e6-5611-51b2-a2b3-6fb9d9aa9566"
private const val RESET_POISON_UUID = "c0d915c8-26b1-50da-951e-d91bc4d3c5e1"

class RangerBluetoothHandler private constructor
    ( private val bluetoothGatt: BluetoothGatt
    , private val demoStartChar: BluetoothGattCharacteristic
    , private val demoCancelChar: BluetoothGattCharacteristic
    , private val resetPoisonChar: BluetoothGattCharacteristic
    , private val poisonStateChar: BluetoothGattCharacteristic
    , private val ctx: Context
): Closeable
{
    override fun close() {
        bluetoothGatt.close()
    }

    private fun resetPoison() {
        TODO()
    }

    companion object {
        private val bluetoothGattCallback: BluetoothGattCallback = object : BluetoothGattCallback() {
        }

        fun connect(ctx: Context, address: String, onDisconnectCallback: Runnable): Future<RangerBluetoothHandler> {
            val bm = ctx.getSystemService(BLUETOOTH_SERVICE)
            if (bm !is BluetoothManager) {
                return CompletableFuture.failedFuture(IllegalStateException("Bluetooth service is not BluetoothManager"))
            }
            val a = bm.adapter
            val device = a.getRemoteDevice(address)

            val h: CompletableFuture<RangerBluetoothHandler> = CompletableFuture()

            val bgf: CompletableFuture<BluetoothGatt> = CompletableFuture()

            val bg = device.connectGatt(ctx, false, object : BluetoothGattCallback() {
                override fun onConnectionStateChange(
                    gatt: BluetoothGatt?,
                    status: Int,
                    newState: Int
                ) {
                    Log.d(TAG, "Connection state change")
                    if (status != BluetoothGatt.GATT_SUCCESS) {
                        Handler(Looper.getMainLooper()).post {
                            Toast.makeText(
                                ctx,
                                "Unable to connect. Make sure the robot is turned on and in range",
                                Toast.LENGTH_LONG
                            ).show()
                        }
                        Log.e(TAG, "GATT connection state change did not return success")
                    }
                    if (newState == BluetoothProfile.STATE_CONNECTED) {
                        Log.i(TAG, "Connected")

                        bgf.get().discoverServices()
                    } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                        onDisconnectCallback.run()
                    }
                }

                override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
                    Log.d(TAG, "Services discovered")
                    if (status == BluetoothGatt.GATT_SUCCESS) {
                        val s = rangerService(gatt)

                        if (s == null) {
                            Log.e(TAG, "Device did not have required services, disconnecting")
                            bgf.get().close()
                            h.completeExceptionally(RuntimeException("Services not found on device"))
                        }
                    }
                }

                override fun onCharacteristicChanged(
                    gatt: BluetoothGatt,
                    characteristic: BluetoothGattCharacteristic,
                    value: ByteArray
                ) {
                    if (characteristic.uuid.toString() == POISON_STATE_UUID) {
                        Log.w(TAG, "Poison state active! Attempting to reset...")
                        h.get().resetPoison()
                    }
                }
            })

            // tying the knot
            bgf.complete(bg)

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
    }
}