package com.example.sdpapp

import android.content.BroadcastReceiver
import android.content.ComponentName
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.ServiceConnection
import android.content.pm.PackageManager
import android.os.Bundle
import android.os.IBinder
import android.util.Log
import android.Manifest
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.NotificationManager.*
import android.os.Build
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.annotation.RequiresApi
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import com.example.sdpapp.bt.RangerBluetoothService
import androidx.lifecycle.viewmodel.compose.viewModel
import com.example.sdpapp.ui.BottomNavigationBar
import com.example.sdpapp.ui.ThemeViewModelFactory
import com.example.sdpapp.ui.theme.SDPAppTheme
import com.example.sdpapp.ui.theme.ThemeViewModel


class MainActivity : ComponentActivity() {
    var bluetoothService : RangerBluetoothService? = null

    private val serviceConnection: ServiceConnection = object : ServiceConnection {
        override fun onServiceConnected(cn: ComponentName?, bnd: IBinder?) {
            Log.d("MainActivity", "Bluetooth service connected")
            bluetoothService = (bnd as RangerBluetoothService.LocalBinder).getService()
            bluetoothService?.let { bluetooth ->
                if (!bluetooth.initialize()) {
                    Log.e("MainActivity", "Unable to initialize Bluetooth")
                    finish()
                }
            }
        }

        override fun onServiceDisconnected(cn: ComponentName?) {
            Log.e("MainActivity", "Bluetooth service disconnected")
            bluetoothService = null
        }
    }

    private val requestNotificationPermissionLauncher =
        registerForActivityResult(ActivityResultContracts.RequestPermission()) { isGranted ->
            if (isGranted) {
                Log.i("MainActivity", "Notification permission granted")
            } else {
                Log.i("MainActivity", "Notification permission denied")
            }
        }

    private fun requestNotificationPermission() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            when {
                ContextCompat.checkSelfPermission(
                    this, Manifest.permission.POST_NOTIFICATIONS
                ) == PackageManager.PERMISSION_GRANTED -> {
                    Log.i("MainActivity", "Notification permission already granted")
                }
                else -> {
                    requestNotificationPermissionLauncher.launch(Manifest.permission.POST_NOTIFICATIONS)
                }
            }
        }
    }


    public val gattUpdateReceiver: BroadcastReceiver = object : BroadcastReceiver() {
        override fun onReceive(ctx: Context, intent: Intent) {
            when (intent.action) {
                RangerBluetoothService.ACTION_GATT_CONNECTED -> {
                    Log.i("MainActivity", "Connected to GATT device")
                }
                RangerBluetoothService.ACTION_GATT_DISCONNECTED -> {
                    Log.w("MainActivity", "Disconnected from GATT device")
                    // Only reconnect if needed
                }
                RangerBluetoothService.ACTION_GATT_READY -> {
                    Log.i("MainActivity", "GATT device ready for startDemo command")
                    // Now, the user should manually press "Start Demo"
                }
            }
        }
    }

    override fun onResume() {
        super.onResume()
        registerReceiverSafely()
    }

    override fun onPause() {
        super.onPause()
        unregisterReceiver(gattUpdateReceiver)
    }

    internal fun registerReceiverSafely() {
        try {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                registerReceiver(gattUpdateReceiver, makeGattUpdateIntentFilter(), Context.RECEIVER_NOT_EXPORTED)
            } else {
                registerReceiver(gattUpdateReceiver, makeGattUpdateIntentFilter())
            }
        } catch (e: IllegalArgumentException) {
            Log.e("MainActivity", "Receiver already registered or error: ${e.message}")
        }
    }

    public fun makeGattUpdateIntentFilter(): IntentFilter {
        return IntentFilter().apply {
            addAction(RangerBluetoothService.ACTION_GATT_CONNECTED)
            addAction(RangerBluetoothService.ACTION_GATT_DISCONNECTED)
            addAction(RangerBluetoothService.ACTION_GATT_READY)
        }
    }

    @RequiresApi(Build.VERSION_CODES.TIRAMISU)
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        bluetoothService?.createNotificationChannel()
        setContent {
            val themeViewModel: ThemeViewModel = viewModel(factory = ThemeViewModelFactory(applicationContext))
            val darkTheme by themeViewModel.darkTheme.collectAsState()

            SDPAppTheme(darkTheme = darkTheme) {
                BottomNavigationBar(themeViewModel, bluetoothService)
            }
        }

        requestNotificationPermission()

        val gattServiceIntent = Intent(this, RangerBluetoothService::class.java)
        Log.d("MainActivity", "Binding service")
        val b = bindService(gattServiceIntent, serviceConnection, Context.BIND_AUTO_CREATE)
        Log.d("MainActivity", "Bluetooth service binding result: $b")
    }

    private val requestBluetoothPermissionLauncher =

        registerForActivityResult(ActivityResultContracts.RequestPermission()) { isGranted ->
            if (isGranted) {
                Log.i("MainActivity", "Bluetooth permission granted")
                bluetoothService?.connectForDemo()
            } else {
                Log.i("MainActivity", "Bluetooth permission denied")
            }
        }

    public fun requestBluetoothPermission() {
        when {
            ContextCompat.checkSelfPermission(
                this,
                Manifest.permission.BLUETOOTH_CONNECT
            ) == PackageManager.PERMISSION_GRANTED -> {
                Log.i("MainActivity", "Bluetooth permission already granted")
                bluetoothService?.connectForDemo()
            }
            else -> {
                ActivityCompat.requestPermissions(this,
                    arrayOf(Manifest.permission.BLUETOOTH_CONNECT), 4444444)
                requestBluetoothPermissionLauncher.launch(Manifest.permission.BLUETOOTH_CONNECT)
            }
        }
    }
}