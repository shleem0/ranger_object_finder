package com.example.sdpapp.ui

import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.ElevatedButton
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.LocalTextStyle
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedCard
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.PlatformTextStyle
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.style.LineHeightStyle
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.em
import androidx.compose.ui.unit.sp
import androidx.navigation.NavController

@Composable
fun AboutSettingsScreen(navController: NavController) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(MaterialTheme.colorScheme.background)
            .padding(20.dp)
    ) {
        Text(
            text = "About The App",
            style = MaterialTheme.typography.bodyLarge,
            color = MaterialTheme.colorScheme.tertiary
        )
        OutlinedCard(
            colors = CardDefaults.cardColors(
                containerColor = MaterialTheme.colorScheme.onBackground,
            ),
            border = BorderStroke(2.dp, MaterialTheme.colorScheme.tertiary),
            modifier = Modifier
                .fillMaxWidth()
                .height(330.dp)
                .padding(top = 20.dp)
        ) {
            Text(
                text = "Pani Pani Pani Pani PANI PANI PANI pani PANI PANI",
                modifier = Modifier
                    .padding(12.dp),
                fontSize = 20.sp,
                color = MaterialTheme.colorScheme.secondary,
                textAlign = TextAlign.Left,
                style = LocalTextStyle.current.merge(
                    TextStyle(
                        lineHeight = 1.em,
                        platformStyle = PlatformTextStyle(
                            includeFontPadding = false
                        ),
                        lineHeightStyle = LineHeightStyle(
                            alignment = LineHeightStyle.Alignment.Center,
                            trim = LineHeightStyle.Trim.None
                        )
                    )
                )
            )
        }
        Column(
            verticalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            HorizontalDivider(
                thickness = 2.dp,
                color = MaterialTheme.colorScheme.surfaceBright,
                modifier = Modifier
                    .padding(top = 15.dp, bottom = 15.dp)
            )
        }
        OutlinedCard(
            colors = CardDefaults.cardColors(
                containerColor = MaterialTheme.colorScheme.onBackground,
            ),
            border = BorderStroke(2.dp, MaterialTheme.colorScheme.tertiary),
            modifier = Modifier
                .fillMaxWidth()
                .height(330.dp)
        ) {
            Text(
                text = "Pani Pani Pani Pani PANI PANI PANI PANI pani pani",
                modifier = Modifier
                    .padding(12.dp),
                fontSize = 20.sp,
                color = MaterialTheme.colorScheme.secondary,
                textAlign = TextAlign.Left,
                style = LocalTextStyle.current.merge(
                    TextStyle(
                        lineHeight = 1.em,
                        platformStyle = PlatformTextStyle(
                            includeFontPadding = false
                        ),
                        lineHeightStyle = LineHeightStyle(
                            alignment = LineHeightStyle.Alignment.Center,
                            trim = LineHeightStyle.Trim.None
                        )
                    )
                )
            )
        }
    }
}
