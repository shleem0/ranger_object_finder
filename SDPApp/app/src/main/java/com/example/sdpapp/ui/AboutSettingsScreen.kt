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
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.material3.Card
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.ElevatedButton
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.LocalTextStyle
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedCard
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
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
    LazyColumn(
        modifier = Modifier
            .fillMaxSize()
            .background(MaterialTheme.colorScheme.background)
            .padding(horizontal = 20.dp)
    ) {
        item {
            Column {
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
                    text = "About The App",
                    style = MaterialTheme.typography.bodyLarge,
                    color = MaterialTheme.colorScheme.tertiary,
                    fontSize = 45.sp
                )
                OutlinedCard(
                    colors = CardDefaults.cardColors(
                        containerColor = MaterialTheme.colorScheme.onBackground,
                    ),
                    border = BorderStroke(2.dp, MaterialTheme.colorScheme.tertiary),
                    modifier = Modifier
                        .fillMaxWidth()
                        .height(330.dp)
                        .padding(top = 16.dp)
                ) {
                    Text(
                        text = "With Ranger, you can find your lost items with ease, whether you " +
                                "are rushing to find your keys in the morning, or trying to locate" +
                                " the forever-lost remote, we have got the solution for you. " +
                                "This app is designed as the user interface for the Ranger robot, " +
                                "which can be bought separately. " +
                                "Add a commonly lost item with at least 3 photos for that item, " +
                                "and the robot will find the object for you.",
                        modifier = Modifier.padding(12.dp),
                        fontSize = 19.sp,
                        color = MaterialTheme.colorScheme.surfaceBright,
                        textAlign = TextAlign.Left,
                        style = LocalTextStyle.current.merge(
                            TextStyle(
                                lineHeight = 22.sp,
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
                            .padding(top = 7.dp, bottom = 7.dp)
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
                        text = "Ranger does searching and finding and picking and retrieving and " +
                                "returning and looking and analysing and Pani.",
                        modifier = Modifier.padding(12.dp),
                        fontSize = 19.sp,
                        color = MaterialTheme.colorScheme.surfaceBright,
                        textAlign = TextAlign.Left,
                        style = LocalTextStyle.current.merge(
                            TextStyle(
                                lineHeight = 22.sp,
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
    }
}