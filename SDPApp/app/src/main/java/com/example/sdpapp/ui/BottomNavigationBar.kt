package com.example.sdpapp.ui

import android.os.Build
import androidx.annotation.RequiresApi
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.padding
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Face
import androidx.compose.material.icons.filled.Home
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material.icons.filled.ShoppingCart
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.NavigationBar
import androidx.compose.material3.NavigationBarItem
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.platform.LocalContext
import androidx.navigation.NavGraph.Companion.findStartDestination
import androidx.navigation.compose.NavHost
import androidx.navigation.compose.composable
import androidx.navigation.compose.rememberNavController
import com.example.sdpapp.SettingsScreen
import com.example.sdpapp.bt.RangerBluetoothService
import com.example.sdpapp.ui.theme.ThemeViewModel

@RequiresApi(Build.VERSION_CODES.TIRAMISU)
@Composable
fun BottomNavigationBar(themeViewModel: ThemeViewModel, bluetoothService: RangerBluetoothService?) {
    var navigationSelectedItem by remember { mutableStateOf(0) }
    val navController = rememberNavController()

    val bottomNavigationItems = listOf(
        NavigationItem("Home", "home", Icons.Filled.Home),
        NavigationItem("Photos", "photos", Icons.Filled.Face),
        NavigationItem("Settings", "settings", Icons.Filled.Settings),
        //NavigationItem("Demo", "demo", Icons.Filled.ShoppingCart)
    )

    Scaffold(
        bottomBar = {
            NavigationBar(
                containerColor = MaterialTheme.colorScheme.background
            ) {
                bottomNavigationItems.forEachIndexed { index, navigationItem ->
                    NavigationBarItem(
                        selected = index == navigationSelectedItem,
                        label = { Text(navigationItem.label) },
                        icon = {
                            Icon(
                                navigationItem.icon,
                                contentDescription = navigationItem.label,
                                tint = if (index == navigationSelectedItem) {
                                    MaterialTheme.colorScheme.surfaceBright
                                } else {
                                    MaterialTheme.colorScheme.onSurface
                                }
                            )
                        },
                        onClick = {
                            navigationSelectedItem = index
                            navController.navigate(navigationItem.route) {
                                popUpTo(navController.graph.findStartDestination().id) {
                                    saveState = true
                                }
                                launchSingleTop = true
                                restoreState = true
                            }
                        },
                    )
                }
            }
        }
    ) { paddingValues ->
        NavHost(
            navController = navController,
            startDestination = bottomNavigationItems[0].route,
            modifier = Modifier.padding(paddingValues)
        ) {
            composable("home") { HomeScreen(navController) }
            composable("photos") { PhotosScreen(navController) }
            composable("settings") { SettingsScreen(navController) }
            composable("about") { AboutSettingsScreen(navController) }
            composable("display") { DisplaySettingsScreen(navController, themeViewModel) }
            composable("permissions") { PermissionsSettingsScreen(navController) }
            composable("alerts") { AlertsScreen(navController) }
            composable("camera/{name}") { backStackEntry ->
                val name = backStackEntry.arguments?.getString("name") ?: ""
                CameraScreen(navController, name)
            }
            composable("cameraPreview") { backStackEntry ->
                val name = backStackEntry.arguments?.getString("name") ?: ""
                CameraPreview(navController, name)
            }
            composable("fullScreenAlert/{alertId}") { backStackEntry ->
                val alertId = backStackEntry.arguments?.getString("alertId")?.toLong() ?: 0L
                FullScreenAlertScreen(navController, alertId, LocalContext.current)
            }
            composable("addItem") { AddItem(navController) }
            composable("deleteItem/{itemName}") { backStackEntry ->
                val itemName = backStackEntry.arguments?.getString("itemName") ?: ""
                DeleteRow(navController, itemName, LocalContext.current)
            }
            composable("search") { SearchScreen(navController) }
            composable("demo") { DemoScreen(navController) }
            composable("openAppSettings") { openAppSettings() }
            composable("connectForDemo") {
                if (bluetoothService != null) {
                    bluetoothService.connectForDemo()
                }
            }
        }
    }
}

data class NavigationItem(
    val label: String,
    val route: String,
    val icon: ImageVector
)