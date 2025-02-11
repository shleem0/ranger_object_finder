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
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
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
                bluetooth.connectForDemo()
            }
        }

        override fun onServiceDisconnected(cn: ComponentName?) {
            bluetoothService = null
        }
    }

    private val gattUpdateReceiver: BroadcastReceiver = object : BroadcastReceiver() {
        override fun onReceive(ctx: Context, it: Intent) {
            when (intent.action) {
                RangerBluetoothService.Companion.ACTION_GATT_CONNECTED -> {
                    Log.i("MainActivity", "Connected to GATT device")
                }
                RangerBluetoothService.Companion.ACTION_GATT_DISCONNECTED -> {
                    Log.w("MainActivity", "Disconnected from GATT device")
                }
                RangerBluetoothService.Companion.ACTION_GATT_READY -> {
                    Log.i("MainActivity", "GATT device ready for startDemo command")
                }
            }
        }
    }

    override fun onResume() {
        super.onResume()
        registerReceiver(gattUpdateReceiver, makeGattUpdateIntentFilter())
        val s = bluetoothService
        if (s != null) {
            val result = s.connectForDemo()
            Log.d("MainActivity", "Connect request result=$result")
        }
    }

    override fun onPause() {
        super.onPause()
        unregisterReceiver(gattUpdateReceiver)
    }

    private fun makeGattUpdateIntentFilter(): IntentFilter {
        return IntentFilter().apply {
            addAction(RangerBluetoothService.Companion.ACTION_GATT_CONNECTED)
            addAction(RangerBluetoothService.Companion.ACTION_GATT_DISCONNECTED)
            addAction(RangerBluetoothService.Companion.ACTION_GATT_READY)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            val themeViewModel: ThemeViewModel = viewModel(factory = ThemeViewModelFactory(applicationContext)) // Pass context
            val darkTheme by themeViewModel.darkTheme.collectAsState()

            SDPAppTheme(darkTheme = darkTheme) {
                BottomNavigationBar(themeViewModel)
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