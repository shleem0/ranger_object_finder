package com.example.sdpapp

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Home
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.ElevatedButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.navigation.NavController
import androidx.navigation.NavGraph.Companion.findStartDestination
import androidx.navigation.compose.NavHost
import androidx.navigation.compose.rememberNavController
import com.example.sdpapp.ui.AboutSettingsScreen
import com.example.sdpapp.ui.NavigationItem

@Composable
fun SettingsScreen(navController: NavController) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(MaterialTheme.colorScheme.background)
            .padding(30.dp)
    ) {
        Text(
            text = "Settings",
            style = MaterialTheme.typography.bodyLarge,
            color = MaterialTheme.colorScheme.surfaceBright
        )
        ElevatedButton(
            onClick = { navController.navigate("display") },
            modifier = Modifier
                .padding(top = 30.dp)
                .fillMaxWidth()
                .height(80.dp)
        ) {
            Text(
                text = "Display",
                fontSize = 26.sp,
                color = MaterialTheme.colorScheme.secondary
            )
        }
        ElevatedButton(
            onClick = { navController.navigate("permissions") },
            modifier = Modifier
                .padding(top = 30.dp)
                .fillMaxWidth()
                .height(80.dp)
        ) {
            Text(
                text = "Permissions",
                fontSize = 25.sp,
                color = MaterialTheme.colorScheme.secondary
            )
        }
        ElevatedButton(
            onClick = { navController.navigate("about") },
            modifier = Modifier
                .padding(top = 30.dp)
                .fillMaxWidth()
                .height(80.dp)
        ) {
            Text(
                text = "About",
                fontSize = 26.sp,
                color = MaterialTheme.colorScheme.secondary
            )
        }
    }
}