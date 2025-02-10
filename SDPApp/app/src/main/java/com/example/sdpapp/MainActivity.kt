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
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import com.example.sdpapp.ui.BottomNavigationBar
import com.example.sdpapp.ui.theme.SDPAppTheme

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
        setContent {
            SDPAppTheme(darkTheme = false) {
                BottomNavigationBar()
            }
        }
        val gattServiceIntent = Intent(this, RangerBluetoothService::class.java)
        bindService(gattServiceIntent, serviceConnection, Context.BIND_AUTO_CREATE)
    }

    private val requestPermissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestPermission()
    ) { isGranted ->
        if (isGranted) {
            Log.i("MainActivity", "Permission granted")
        } else {
            Log.i("MainActivity", "Permission denied")
        }
    }

    private fun requestCameraPermission() {
        when {
            ContextCompat.checkSelfPermission(
                this,
                android.Manifest.permission.CAMERA
            ) == PackageManager.PERMISSION_GRANTED -> {
                Log.i("MainActivity", "Permission granted before")
            }

            ActivityCompat.shouldShowRequestPermissionRationale(
                this,
                android.Manifest.permission.CAMERA
            ) -> Log.i("MainActivity", "Show camera permissions rationale")

            else -> requestPermissionLauncher.launch(android.Manifest.permission.CAMERA)
        }
    }
}