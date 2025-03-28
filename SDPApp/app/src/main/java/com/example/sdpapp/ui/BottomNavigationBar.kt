package com.example.sdpapp.ui

import android.os.Build
import androidx.activity.ComponentActivity
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
import androidx.navigation.compose.currentBackStackEntryAsState
import androidx.navigation.compose.rememberNavController
import com.example.sdpapp.PermissionManager
import com.example.sdpapp.SettingsScreen
import com.example.sdpapp.bt.RangerBluetoothService
import com.example.sdpapp.ui.theme.ThemeViewModel

@RequiresApi(Build.VERSION_CODES.TIRAMISU)
@Composable
fun BottomNavigationBar(
    themeViewModel: ThemeViewModel,
    bluetoothService: RangerBluetoothService?,
    permissionManager: PermissionManager
){
    val navController = rememberNavController()
    val currentBackStackEntry by navController.currentBackStackEntryAsState()
    val currentRoute = currentBackStackEntry?.destination?.route

    val bottomNavigationItems = listOf(
        NavigationItem("Home", "home", Icons.Filled.Home),
        NavigationItem("Photos", "photos", Icons.Filled.Face),
        NavigationItem("Settings", "settings", Icons.Filled.Settings)
    )

    Scaffold(
        bottomBar = {
            NavigationBar(containerColor = MaterialTheme.colorScheme.background) {
                bottomNavigationItems.forEach { navigationItem ->
                    NavigationBarItem(
                        selected = currentRoute?.startsWith(navigationItem.route) == true,
                        label = { Text(navigationItem.label) },
                        icon = {
                            Icon(
                                navigationItem.icon,
                                contentDescription = navigationItem.label,
                                tint = if (currentRoute?.startsWith(navigationItem.route) == true) {
                                    MaterialTheme.colorScheme.surfaceBright
                                } else {
                                    MaterialTheme.colorScheme.onSurface
                                }
                            )
                        },
                        onClick = {
                            if (currentRoute == navigationItem.route) {
                                navController.popBackStack(navigationItem.route, inclusive = true)
                                navController.navigate(navigationItem.route)
                            } else {
                                navController.navigate(navigationItem.route) {
                                    popUpTo(navController.graph.findStartDestination().id) {
                                        saveState = true
                                    }
                                    launchSingleTop = true
                                    restoreState = true
                                }
                            }
                        },
                    )
                }
            }
        }
    )  { paddingValues ->
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
            composable("permissions") {
                PermissionsSettingsScreen(navController, permissionManager = permissionManager)
            }
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
            composable("connectForDemo") {
                if (bluetoothService != null) {
                    bluetoothService.connectForDemo()
                }
            }
            composable("iconSelection/{itemName}") { backStackEntry ->
                val itemName = backStackEntry.arguments?.getString("itemName") ?: ""
                iconSelection(navController, itemName)
            }
            composable("upload/{itemName}") { backStackEntry ->
                val itemName = backStackEntry.arguments?.getString("itemName") ?: ""
                UploadScreen(navController, itemName)
            }
            composable("uploadContent/{itemName}") { backStackEntry ->
                val itemName = backStackEntry.arguments?.getString("itemName") ?: ""
                UploadContent(navController, itemName)
            }
            composable("deniedPermission") { PermissionDeniedView(LocalContext.current) }
        }
    }
}

data class NavigationItem(
    val label: String,
    val route: String,
    val icon: ImageVector
)