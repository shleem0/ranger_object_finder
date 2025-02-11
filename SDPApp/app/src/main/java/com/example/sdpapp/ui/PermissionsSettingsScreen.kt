@file:OptIn(ExperimentalPermissionsApi::class)

package com.example.sdpapp.ui

import android.Manifest
import android.content.Intent
import android.net.Uri
import android.provider.Settings
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
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
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
        modifier = Modifier.fillMaxWidth().padding(top = 14.dp),
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
                    // If enabling, request permission
                    cameraPermissionState.launchPermissionRequest()
                } else {
                    // If disabling, guide user to settings since permissions cannot be revoked programmatically
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