package com.example.sdpapp

import android.content.BroadcastReceiver
import android.content.ComponentName
import android.content.Context
import android.content.Intent
import android.content.ServiceConnection
import android.content.pm.PackageManager
import android.os.Bundle
import android.os.IBinder
import android.util.Log
import android.Manifest
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import androidx.lifecycle.viewmodel.compose.viewModel
import com.example.sdpapp.ui.BottomNavigationBar
import com.example.sdpapp.ui.ThemeViewModelFactory
import com.example.sdpapp.ui.theme.SDPAppTheme
import com.example.sdpapp.ui.theme.ThemeViewModel

// placeholder, should replace this with the Raspberry Pi's address and maybe
// make it automatically find the device by the available service UUIDs
// see example https://github.com/bluez/bluer/blob/master/bluer/examples/gatt_client.rs
private const val RANGER_ADDRESS = "90:32:4B:8F:FC:D6"

class MainActivity : ComponentActivity() {
    private var bluetoothService : RangerBluetoothService? = null

    private val serviceConnection: ServiceConnection = object : ServiceConnection {
        override fun onServiceConnected(cn: ComponentName?, bnd: IBinder?) {
            bluetoothService = (bnd as RangerBluetoothService.LocalBinder).getService()
            bluetoothService?.let { bluetooth ->
                if (!bluetooth.initialize()) {
                    Log.e("MainActivity", "Unable to initialize Bluetooth")
                    // may want to show the user a failure message instead of this
                    finish()
                }
                bluetooth.connect(RANGER_ADDRESS)
            }
        }

        override fun onServiceDisconnected(cn: ComponentName?) {
            bluetoothService = null
        }
    }

    private val gattUpdateReceiver: BroadcastReceiver = object : BroadcastReceiver() {
        override fun onReceive(ctx: Context, it: Intent) {
            when (intent.action) {
                // TODO
            }
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // Check Bluetooth permissions first
        requestBluetoothPermissions()

        setContent {
            val themeViewModel: ThemeViewModel = viewModel(factory = ThemeViewModelFactory(applicationContext))
            val darkTheme by themeViewModel.darkTheme.collectAsState()

            SDPAppTheme(darkTheme = darkTheme) {
                BottomNavigationBar(themeViewModel, bluetoothService)
            }
        }
    }

    private fun requestBluetoothPermissions() {
        when {
            ContextCompat.checkSelfPermission(
                this, Manifest.permission.BLUETOOTH_CONNECT
            ) == PackageManager.PERMISSION_GRANTED -> {
                Log.i("MainActivity", "Bluetooth permission already granted")
                startBluetoothService()
            }

            ActivityCompat.shouldShowRequestPermissionRationale(this, Manifest.permission.BLUETOOTH_CONNECT) -> {
                Log.i("MainActivity", "Showing Bluetooth permission rationale")
                requestBluetoothPermissionLauncher.launch(Manifest.permission.BLUETOOTH_CONNECT)
            }

            else -> {
                requestBluetoothPermissionLauncher.launch(Manifest.permission.BLUETOOTH_CONNECT)
            }
        }
    }

    private val requestBluetoothPermissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { isGranted ->
        if (isGranted) {
            Log.i("MainActivity", "Bluetooth permission granted")
            startBluetoothService()
        } else {
            Log.e("MainActivity", "Bluetooth permission denied")
        }
    }

    private fun startBluetoothService() {
        val serviceIntent = Intent(this, RangerBluetoothService::class.java)
        bindService(serviceIntent, serviceConnection, Context.BIND_AUTO_CREATE)
    }

}