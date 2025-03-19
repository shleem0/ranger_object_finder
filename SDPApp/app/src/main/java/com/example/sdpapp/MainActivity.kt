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
import android.os.Build
import android.Manifest
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.annotation.RequiresApi
import androidx.compose.runtime.LaunchedEffect
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
import kotlinx.coroutines.delay


@RequiresApi(Build.VERSION_CODES.TIRAMISU)
class MainActivity : ComponentActivity() {
    var bluetoothService: RangerBluetoothService? = null
    private lateinit var permissionManager: PermissionManager

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        permissionManager = PermissionManager(this)

        setContent {
            val themeViewModel: ThemeViewModel = viewModel(factory = ThemeViewModelFactory(applicationContext))
            val darkTheme by themeViewModel.darkTheme.collectAsState()

            SDPAppTheme(darkTheme = darkTheme) {
                BottomNavigationBar(themeViewModel, bluetoothService, permissionManager)
            }

            // Keep checking and requesting permissions
            LaunchedEffect(Unit) {
                ensurePermissionsGranted()
            }
        }

        bindBluetoothService()
    }

    private suspend fun ensurePermissionsGranted() {
        while (true) {
            delay(1000) // Retry every 1 second

            if (!hasRequiredPermissions()) {
                requestAllPermissions()
            } else {
                Log.i("MainActivity", "All permissions granted ✅")
                break
            }
        }
    }

    private fun hasRequiredPermissions(): Boolean {
        return ContextCompat.checkSelfPermission(
                    this, Manifest.permission.POST_NOTIFICATIONS
                ) == PackageManager.PERMISSION_GRANTED &&
                ContextCompat.checkSelfPermission(
                    this, Manifest.permission.BLUETOOTH_CONNECT
                ) == PackageManager.PERMISSION_GRANTED
    }

    private fun requestAllPermissions() {
        permissionManager.requestNotificationPermission()
        permissionManager.requestBluetoothPermission()
    }

    private fun bindBluetoothService() {
        val gattServiceIntent = Intent(this, RangerBluetoothService::class.java)
        bindService(gattServiceIntent, serviceConnection, Context.BIND_AUTO_CREATE)
    }

    private val serviceConnection = object : ServiceConnection {
        override fun onServiceConnected(name: ComponentName?, service: IBinder?) {
            bluetoothService = (service as RangerBluetoothService.LocalBinder).getService()
            bluetoothService?.let { bluetooth ->
                if (!bluetooth.initialize()) {
                    Log.e("MainActivity", "Unable to initialize Bluetooth")
                    finish()
                }
            }
        }

        override fun onServiceDisconnected(name: ComponentName?) {
            bluetoothService = null
        }
    }

    private val bluetoothReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context?, intent: Intent?) {
            when (intent?.action) {
                BluetoothDevice.ACTION_ACL_CONNECTED -> {
                    Log.i("MainActivity", "Bluetooth device connected")
                }
                BluetoothDevice.ACTION_ACL_DISCONNECTED -> {
                    Log.i("MainActivity", "Bluetooth device disconnected")
                }
                BluetoothAdapter.ACTION_STATE_CHANGED -> {
                    val state = intent.getIntExtra(BluetoothAdapter.EXTRA_STATE, BluetoothAdapter.ERROR)
                    when (state) {
                        BluetoothAdapter.STATE_ON -> Log.i("MainActivity", "Bluetooth is ON")
                        BluetoothAdapter.STATE_OFF -> Log.i("MainActivity", "Bluetooth is OFF")
                    }
                }
            }
        }
    }

    fun registerReceiverSafely() {
        val filter = IntentFilter().apply {
            addAction(BluetoothDevice.ACTION_ACL_CONNECTED)
            addAction(BluetoothDevice.ACTION_ACL_DISCONNECTED)
            addAction(BluetoothAdapter.ACTION_STATE_CHANGED)
        }

        try {
            registerReceiver(bluetoothReceiver, filter)
            Log.i("MainActivity", "Bluetooth receiver registered ✅")
        } catch (e: Exception) {
            Log.e("MainActivity", "Failed to register Bluetooth receiver: ${e.message}")
        }
    }

    override fun onStart() {
        super.onStart()

        // Register the Bluetooth broadcast receiver
        val filter = IntentFilter().apply {
            addAction(BluetoothDevice.ACTION_ACL_CONNECTED)
            addAction(BluetoothDevice.ACTION_ACL_DISCONNECTED)
            addAction(BluetoothAdapter.ACTION_STATE_CHANGED)
        }

        try {
            registerReceiver(bluetoothReceiver, filter)
            Log.i("MainActivity", "Bluetooth receiver registered ✅")
        } catch (e: Exception) {
            Log.e("MainActivity", "Failed to register Bluetooth receiver: ${e.message}")
        }
    }

    override fun onStop() {
        super.onStop()

        try {
            unregisterReceiver(bluetoothReceiver)
            Log.i("MainActivity", "Bluetooth receiver unregistered ✅")
        } catch (e: IllegalArgumentException) {
            Log.e("MainActivity", "Failed to unregister Bluetooth receiver: ${e.message}")
        }
    }
}