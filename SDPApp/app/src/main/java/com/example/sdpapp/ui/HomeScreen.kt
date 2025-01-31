package com.example.sdpapp

import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Add
import androidx.compose.material.icons.filled.Notifications
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.navigation.NavController
import androidx.compose.material3.ExtendedFloatingActionButton
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.contentColorFor
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.res.stringResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.navigation.compose.NavHost
import androidx.navigation.compose.composable
import androidx.navigation.compose.rememberNavController
import com.example.sdpapp.ui.AboutSettingsScreen
import com.example.sdpapp.ui.AlertsScreen
import com.example.sdpapp.ui.DisplaySettingsScreen
import com.example.sdpapp.ui.PermissionsSettingsScreen
import com.example.sdpapp.ui.theme.SDPAppTheme

@Composable
fun HomeScreen(navController: NavController) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(MaterialTheme.colorScheme.background)
            .padding(20.dp)
    ) {
        Image(
            painter = painterResource(id = R.drawable.rangergreenletters),
            contentDescription = stringResource(id = R.string.logo_description)
        )
        Text(
            text = "Items",
            color = MaterialTheme.colorScheme.surfaceBright,
            fontSize = 30.sp,
            fontWeight = FontWeight.Bold,
            modifier = Modifier.padding(vertical = 10.dp)
        )
        Item(navController, "keys")
        Item(navController, "remote")
        Item(navController, "wallet")
        FABWithNotification(1, navController)
    }
}

@Composable
fun Item (navController: NavController, name: String){
    val imageName = "sdp$name"
    val context = LocalContext.current
    val imageResId = remember {
        context.resources.getIdentifier(imageName, "drawable", context.packageName)
    }

    Row(
        modifier = Modifier
            .fillMaxWidth()
            .padding(bottom = 10.dp)
            .height(60.dp),
        horizontalArrangement = Arrangement.SpaceBetween,
        verticalAlignment = Alignment.CenterVertically
    ) {
        Row(
            verticalAlignment = Alignment.CenterVertically
        ) {
            Image(
                painter = painterResource(id = imageResId),
                contentDescription = stringResource(id = R.string.key_description),
                modifier = Modifier.size(50.dp)
            )
            Spacer(modifier = Modifier.width(15.dp))
            Text(
                text = name.replaceFirstChar { it.uppercase() },
                fontSize = 30.sp,
                color = MaterialTheme.colorScheme.surfaceBright
            )
        }
        IconButton(
            onClick = { navController.navigate("camera/$name") }
        ) {
            Icon(
                Icons.Filled.Add,
                contentDescription = "Add " + name + " Photos",
                modifier = Modifier.size(35.dp),
                contentColorFor(MaterialTheme.colorScheme.surfaceBright),
            )
        }
    }
}

@Composable
fun FABWithNotification(notificationCount: Int, navController: NavController) {
    Box(
        modifier = Modifier
            .fillMaxSize()
            .padding(16.dp),
        contentAlignment = Alignment.BottomEnd
    ) {
        Box(
            contentAlignment = Alignment.TopEnd
        ) {
            ExtendedFloatingActionButton(
                onClick = { navController.navigate("alerts") },
                icon = { Icon(Icons.Filled.Notifications, contentDescription = "Alert Button") },
                text = { Text(text = "Alerts") },
                containerColor = MaterialTheme.colorScheme.onBackground,
                contentColor = MaterialTheme.colorScheme.surfaceBright
            )

            if (notificationCount > 0) {
                Box(
                    modifier = Modifier
                        .size(20.dp)
                        .background(Color.Red, shape = CircleShape)
                        .align(Alignment.TopEnd)
                )
            }
        }
    }
}

@Composable
fun CameraScreen(navController: NavController, name: String){
    Text(
        name,
        color = MaterialTheme.colorScheme.surfaceBright
    )
}

@Preview(showBackground = true)
@Composable
fun HomeScreenPreview() {
    SDPAppTheme {
        HomeScreen(navController = rememberNavController())
    }
}