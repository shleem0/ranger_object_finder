@file:OptIn(ExperimentalPermissionsApi::class)

package com.example.sdpapp.ui

import android.Manifest
import android.content.Intent
import android.content.pm.PackageManager
import android.net.Uri
import android.os.Build
import android.provider.Settings
import android.widget.Toast
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Switch
import androidx.compose.material3.SwitchDefaults
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.core.content.ContextCompat
import androidx.navigation.NavController
import com.google.accompanist.permissions.ExperimentalPermissionsApi
import com.google.accompanist.permissions.isGranted
import com.google.accompanist.permissions.rememberPermissionState

@Composable
fun PermissionsSettingsScreen(navController: NavController) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(MaterialTheme.colorScheme.background)
            .padding(horizontal = 20.dp)
    ) {
        TextButton(onClick = { navController.navigate("settings") }) {
            Text("< Back", color = MaterialTheme.colorScheme.surfaceBright, fontSize = 18.sp)
        }

        Text(
            text = "Permissions",
            style = MaterialTheme.typography.bodyLarge,
            color = MaterialTheme.colorScheme.tertiary,
        )

        CameraPermissionSwitch(navController)
        BluetoothPermissionSwitch(navController)
        NotificationPermissionSwitch(navController)
    }
}

@Composable
fun CameraPermissionSwitch(navController: NavController) {
    val cameraPermissionState = rememberPermissionState(Manifest.permission.CAMERA)
    var checked by remember { mutableStateOf(cameraPermissionState.status.isGranted) }

    LaunchedEffect(cameraPermissionState.status.isGranted) {
        checked = cameraPermissionState.status.isGranted
    }

    Row(
        modifier = Modifier.fillMaxWidth().padding(top = 12.dp),
        horizontalArrangement = Arrangement.SpaceBetween
    ) {
        Text(
            text = "Camera",
            fontSize = 26.sp,
            color = MaterialTheme.colorScheme.surfaceBright
        )

        Switch(
            checked = checked,
            onCheckedChange = { newCheckedState ->
                if (newCheckedState) {
                    cameraPermissionState.launchPermissionRequest()
                } else {
                    navController.navigate("openAppSettings")
                }
            },
            colors = SwitchDefaults.colors(
                checkedThumbColor = MaterialTheme.colorScheme.onBackground,
                checkedTrackColor = MaterialTheme.colorScheme.secondary,
                uncheckedThumbColor = MaterialTheme.colorScheme.surfaceBright,
                uncheckedTrackColor = MaterialTheme.colorScheme.onBackground,
            )
        )
    }
    Text("The camera is needed for you to take photos of your items.",
        modifier = Modifier
            .fillMaxWidth(),
        fontSize = 16.sp,
        color = MaterialTheme.colorScheme.surfaceBright,
        textAlign = TextAlign.Start,
        lineHeight = 19.sp
    )
}

@Composable
fun BluetoothPermissionSwitch(navController: NavController) {
    val bluetoothPermissionState = rememberPermissionState(Manifest.permission.BLUETOOTH_CONNECT)
    var checked by remember { mutableStateOf(bluetoothPermissionState.status.isGranted) }

    LaunchedEffect(bluetoothPermissionState.status.isGranted) {
        checked = bluetoothPermissionState.status.isGranted
    }

    Row(
        modifier = Modifier.fillMaxWidth().padding(top = 12.dp),
        horizontalArrangement = Arrangement.SpaceBetween
    ) {
        Text(
            text = "Bluetooth",
            fontSize = 26.sp,
            color = MaterialTheme.colorScheme.surfaceBright
        )

        Switch(
            checked = checked,
            onCheckedChange = { newCheckedState ->
                if (newCheckedState) {
                    bluetoothPermissionState.launchPermissionRequest()
                } else {
                    navController.navigate("openAppSettings")
                }
            },
            colors = SwitchDefaults.colors(
                checkedThumbColor = MaterialTheme.colorScheme.onBackground,
                checkedTrackColor = MaterialTheme.colorScheme.secondary,
                uncheckedThumbColor = MaterialTheme.colorScheme.surfaceBright,
                uncheckedTrackColor = MaterialTheme.colorScheme.onBackground,
            )
        )
    }
    Text("Bluetooth is used to connect to the robot.",
        modifier = Modifier
            .fillMaxWidth(),
        fontSize = 16.sp,
        color = MaterialTheme.colorScheme.surfaceBright,
        textAlign = TextAlign.Start,
        lineHeight = 19.sp
    )
}

@Composable
fun NotificationPermissionSwitch(navController: NavController) {
    val context = LocalContext.current
    var checked by remember { mutableStateOf(false) }

    val requestPermissionLauncher = rememberLauncherForActivityResult(
        contract = ActivityResultContracts.RequestPermission()
    ) { isGranted ->
        checked = isGranted
        if (!isGranted) {
            Toast.makeText(context, "Enable notifications in settings", Toast.LENGTH_SHORT).show()
        }
    }

    LaunchedEffect(Unit) {
        checked = ContextCompat.checkSelfPermission(
            context,
            Manifest.permission.POST_NOTIFICATIONS
        ) == PackageManager.PERMISSION_GRANTED
    }

    Row(
        modifier = Modifier.fillMaxWidth().padding(top = 12.dp),
        horizontalArrangement = Arrangement.SpaceBetween
    ) {
        Text(
            text = "Notifications",
            fontSize = 26.sp,
            color = MaterialTheme.colorScheme.surfaceBright
        )

        Switch(
            checked = checked,
            onCheckedChange = { newCheckedState ->
                if (newCheckedState) {
                    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                        if (ContextCompat.checkSelfPermission(
                                context, Manifest.permission.POST_NOTIFICATIONS
                            ) == PackageManager.PERMISSION_GRANTED
                        ) {
                            checked = true
                        } else {
                            requestPermissionLauncher.launch(Manifest.permission.POST_NOTIFICATIONS)
                        }
                    } else {
                        checked = true
                    }
                } else {
                    val intent = Intent(Settings.ACTION_APPLICATION_DETAILS_SETTINGS).apply {
                        data = Uri.fromParts("package", context.packageName, null)
                        addFlags(Intent.FLAG_ACTIVITY_NEW_TASK)
                    }
                    context.startActivity(intent)
                }
            },
            colors = SwitchDefaults.colors(
                checkedThumbColor = MaterialTheme.colorScheme.onBackground,
                checkedTrackColor = MaterialTheme.colorScheme.secondary,
                uncheckedThumbColor = MaterialTheme.colorScheme.surfaceBright,
                uncheckedTrackColor = MaterialTheme.colorScheme.onBackground,
            )
        )
    }
    Text("We recommend keeping notifications on to receive important alerts.",
        modifier = Modifier
            .fillMaxWidth(),
        fontSize = 16.sp,
        color = MaterialTheme.colorScheme.surfaceBright,
        textAlign = TextAlign.Start,
        lineHeight = 19.sp
    )
}

@Composable
fun openAppSettings() {
    val context = LocalContext.current
    val intent = Intent(Settings.ACTION_APPLICATION_DETAILS_SETTINGS).apply {
        data = Uri.fromParts("package", context.packageName, null)
        addFlags(Intent.FLAG_ACTIVITY_NEW_TASK)
    }
    context.startActivity(intent)
}