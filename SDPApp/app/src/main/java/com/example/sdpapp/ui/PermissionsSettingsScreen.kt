@file:OptIn(ExperimentalPermissionsApi::class)

package com.example.sdpapp.ui

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
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.navigation.NavController
import com.example.sdpapp.PermissionManager
import com.google.accompanist.permissions.ExperimentalPermissionsApi

@Composable
fun PermissionsSettingsScreen(
    navController: NavController,
    permissionManager: PermissionManager
) {
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

        CameraPermissionSwitch(permissionManager)
        BluetoothPermissionSwitch(permissionManager)
        NotificationPermissionSwitch(permissionManager)
        FileAccessPermissionSwitch(permissionManager)
    }
}

@Composable
fun CameraPermissionSwitch(permissionManager: PermissionManager) {
    var isGranted by remember { mutableStateOf(permissionManager.isCameraPermissionGranted()) }

    LaunchedEffect(Unit) {
        isGranted = permissionManager.isCameraPermissionGranted()
    }

    Row(
        modifier = Modifier
            .fillMaxWidth()
            .padding(top = 12.dp),
        horizontalArrangement = Arrangement.SpaceBetween
    ) {
        Text(
            text = "Camera",
            fontSize = 26.sp,
            color = MaterialTheme.colorScheme.surfaceBright
        )

        Switch(
            checked = isGranted,
            onCheckedChange = { newCheckedState ->
                if (newCheckedState) {
                    permissionManager.requestCameraPermission()
                } else {
                    permissionManager.openAppSettings()
                }
                isGranted = permissionManager.isCameraPermissionGranted()
            },
            colors = SwitchDefaults.colors(
                checkedThumbColor = MaterialTheme.colorScheme.onBackground,
                checkedTrackColor = MaterialTheme.colorScheme.secondary,
                uncheckedThumbColor = MaterialTheme.colorScheme.surfaceBright,
                uncheckedTrackColor = MaterialTheme.colorScheme.onBackground,
            )
        )
    }

    Text(
        "The camera is needed for you to take photos of your items.",
        modifier = Modifier.fillMaxWidth(),
        fontSize = 16.sp,
        color = MaterialTheme.colorScheme.surfaceBright,
        lineHeight = 18.sp
    )
}

@Composable
fun BluetoothPermissionSwitch(permissionManager: PermissionManager) {
    var isGranted by remember { mutableStateOf(permissionManager.isBluetoothPermissionGranted()) }

    LaunchedEffect(Unit) {
        isGranted = permissionManager.isBluetoothPermissionGranted()
    }

    Row(
        modifier = Modifier
            .fillMaxWidth()
            .padding(top = 12.dp),
        horizontalArrangement = Arrangement.SpaceBetween
    ) {
        Text(
            text = "Bluetooth",
            fontSize = 26.sp,
            color = MaterialTheme.colorScheme.surfaceBright
        )

        Switch(
            checked = isGranted,
            onCheckedChange = { newCheckedState ->
                if (newCheckedState) {
                    permissionManager.requestBluetoothPermission()
                } else {
                    permissionManager.openAppSettings()
                }
                isGranted = permissionManager.isBluetoothPermissionGranted()
            },
            colors = SwitchDefaults.colors(
                checkedThumbColor = MaterialTheme.colorScheme.onBackground,
                checkedTrackColor = MaterialTheme.colorScheme.secondary,
                uncheckedThumbColor = MaterialTheme.colorScheme.surfaceBright,
                uncheckedTrackColor = MaterialTheme.colorScheme.onBackground,
            )
        )
    }

    Text(
        "Bluetooth is used to connect to the robot.",
        modifier = Modifier.fillMaxWidth(),
        fontSize = 16.sp,
        color = MaterialTheme.colorScheme.surfaceBright,
        lineHeight = 18.sp
    )
}

@Composable
fun NotificationPermissionSwitch(permissionManager: PermissionManager) {
    var isGranted by remember { mutableStateOf(permissionManager.isNotificationPermissionGranted()) }

    LaunchedEffect(Unit) {
        isGranted = permissionManager.isNotificationPermissionGranted()
    }

    Row(
        modifier = Modifier
            .fillMaxWidth()
            .padding(top = 12.dp),
        horizontalArrangement = Arrangement.SpaceBetween
    ) {
        Text(
            text = "Notifications",
            fontSize = 26.sp,
            color = MaterialTheme.colorScheme.surfaceBright
        )

        Switch(
            checked = isGranted,
            onCheckedChange = { newCheckedState ->
                if (newCheckedState) {
                    permissionManager.requestNotificationPermission()
                } else {
                    permissionManager.openAppSettings()
                }
                isGranted = permissionManager.isNotificationPermissionGranted()
            },
            colors = SwitchDefaults.colors()
        )
    }

    Text(
        "We recommend keeping notifications on to receive important alerts.",
        modifier = Modifier.fillMaxWidth(),
        fontSize = 16.sp,
        color = MaterialTheme.colorScheme.surfaceBright,
        lineHeight = 18.sp
    )
}

@Composable
fun FileAccessPermissionSwitch(permissionManager: PermissionManager) {
    var isGranted by remember { mutableStateOf(permissionManager.isFilePermissionGranted()) }

    LaunchedEffect(Unit) {
        isGranted = permissionManager.isFilePermissionGranted()
    }

    Row(
        modifier = Modifier
            .fillMaxWidth()
            .padding(top = 12.dp),
        horizontalArrangement = Arrangement.SpaceBetween
    ) {
        Text(
            text = "File Access",
            fontSize = 26.sp,
            color = MaterialTheme.colorScheme.surfaceBright
        )

        Switch(
            checked = isGranted,
            onCheckedChange = { newCheckedState ->
                if (newCheckedState) {
                    permissionManager.requestFilePermissions()
                } else {
                    permissionManager.openAppSettings()
                }
                isGranted = permissionManager.isFilePermissionGranted()
            },
            colors = SwitchDefaults.colors(
                checkedThumbColor = MaterialTheme.colorScheme.onBackground,
                checkedTrackColor = MaterialTheme.colorScheme.secondary,
                uncheckedThumbColor = MaterialTheme.colorScheme.surfaceBright,
                uncheckedTrackColor = MaterialTheme.colorScheme.onBackground,
            )
        )
    }

    Text(
        "File access is recommended to upload photos. You can enable it later if needed.",
        modifier = Modifier.fillMaxWidth(),
        fontSize = 16.sp,
        color = MaterialTheme.colorScheme.surfaceBright,
        lineHeight = 18.sp
    )
}