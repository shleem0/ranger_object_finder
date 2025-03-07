package com.example.sdpapp.ui

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Switch
import androidx.compose.material3.SwitchDefaults
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.LineHeightStyle
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.navigation.NavController
import com.example.sdpapp.ui.theme.SDPAppTheme
import com.example.sdpapp.ui.theme.ThemeViewModel

@Composable
fun DisplaySettingsScreen(navController: NavController, themeViewModel: ThemeViewModel) {
    val darkTheme by themeViewModel.darkTheme.collectAsState()

    SDPAppTheme(darkTheme = darkTheme) {
        Column(
            modifier = Modifier
                .fillMaxSize()
                .background(MaterialTheme.colorScheme.background)
                .padding(horizontal = 20.dp)
        ) {
            TextButton(
                onClick = { navController.navigate("settings") }
            ) {
                Text(
                    "< Back",
                    color = MaterialTheme.colorScheme.surfaceBright,
                    fontSize = 18.sp,
                    modifier = Modifier.padding(0.dp)
                )
            }
            Text(
                text = "Display",
                style = MaterialTheme.typography.bodyLarge,
                color = MaterialTheme.colorScheme.tertiary,
                modifier = Modifier.padding(bottom = 10.dp)
            )

//            Text(
//                text = "Subsection",
//                color = MaterialTheme.colorScheme.surfaceBright,
//                fontSize = 27.sp,
//                fontWeight = FontWeight.Bold,
//                modifier = Modifier.padding(top = 10.dp)
//            )

            Row(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(top = 4.dp),
                horizontalArrangement = Arrangement.Absolute.SpaceBetween
            ) {
                Text(
                    text = "Dark Mode",
                    fontSize = 25.sp,
                    color = MaterialTheme.colorScheme.surfaceBright
                )

                Switch(
                    checked = darkTheme,
                    onCheckedChange = { themeViewModel.toggleTheme() },
                    colors = SwitchDefaults.colors(
                        checkedThumbColor = MaterialTheme.colorScheme.onBackground,
                        checkedTrackColor = MaterialTheme.colorScheme.secondary,
                        uncheckedThumbColor = MaterialTheme.colorScheme.surfaceBright,
                        uncheckedTrackColor = MaterialTheme.colorScheme.onBackground,
                    )
                )
            }

            HorizontalDivider(
                thickness = 1.dp,
                color = MaterialTheme.colorScheme.surfaceBright,
                modifier = Modifier
                    .padding(top = 8.dp, bottom = 8.dp)
            )
        }
    }
}