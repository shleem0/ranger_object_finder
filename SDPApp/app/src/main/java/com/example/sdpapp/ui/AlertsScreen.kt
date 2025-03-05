package com.example.sdpapp.ui

import android.content.Context
import android.os.Build
import androidx.annotation.RequiresApi
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.absoluteOffset
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.LocalTextStyle
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedCard
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.remember
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.PlatformTextStyle
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.style.LineHeightStyle
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.em
import androidx.compose.ui.unit.sp
import androidx.navigation.NavController
import androidx.navigation.compose.rememberNavController
import java.util.Date
import com.google.gson.Gson
import com.google.gson.reflect.TypeToken
import java.text.SimpleDateFormat
import java.time.Instant
import java.time.YearMonth
import java.util.Locale

data class Alert(
    val id: Long,
    val date: Date,
    val title: String,
    val description: String
)

fun saveAlertsToSharedPreferences(context: Context, alerts: List<Alert>) {
    val sharedPreferences = context.getSharedPreferences("alerts", Context.MODE_PRIVATE)
    val editor = sharedPreferences.edit()

    val json = Gson().toJson(alerts)
    editor.putString("alert_list", json)
    editor.apply()
}

fun getAlertsFromSharedPreferences(context: Context): List<Alert> {
    val sharedPreferences = context.getSharedPreferences("alerts", Context.MODE_PRIVATE)
    val json = sharedPreferences.getString("alert_list", "[]")
    val type = object : TypeToken<List<Alert>>() {}.type

    return Gson().fromJson(json, type)
}

@Composable
fun AlertsScreen(navController: NavController) {
    val context = LocalContext.current

    val dateFormat = SimpleDateFormat("yyyy-MM-dd", Locale.UK)
    val parsedDate = dateFormat.parse("2025-02-10")!!
    saveAlertsToSharedPreferences(
        context,
        listOf(Alert(0, parsedDate, "Example Alert", "Description of example alert that will contain informative information and can be opened in full screen if the text runs over the allocated space like this does.")))
    val alerts = remember { getAlertsFromSharedPreferences(context) }

    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(MaterialTheme.colorScheme.background)
            .padding(horizontal = 10.dp)
            .verticalScroll(rememberScrollState())
    ) {
        TextButton(
            onClick = { navController.navigate("home") }
        ) {
            Text(
                "< Back",
                color = MaterialTheme.colorScheme.surfaceBright,
                fontSize = 18.sp,
                modifier = Modifier.padding(0.dp)
            )
        }
        HorizontalDivider(
            thickness = 1.dp,
            color = MaterialTheme.colorScheme.surfaceBright,
            modifier = Modifier.padding(bottom = 8.dp)
        )
        Box(
            contentAlignment = Alignment.Center
        ){
            AlertList(alerts, navController, context)
        }
    }
}

@Composable
fun AlertList(alerts: List<Alert>, navController: NavController, context: Context) {
    if (alerts.isEmpty()){
        Text("No Alerts", fontSize = 20.sp,
            color = MaterialTheme.colorScheme.secondary,
            textAlign = TextAlign.Center,
            modifier = Modifier.padding(start = 10.dp)
        )
    }
    else {
        for (alert in alerts) {
            OutlinedCard(
                colors = CardDefaults.cardColors(
                    containerColor = MaterialTheme.colorScheme.onBackground,
                ),
                border = BorderStroke(2.dp, MaterialTheme.colorScheme.tertiary),
                modifier = Modifier
                    .fillMaxWidth()
                    .height(130.dp)
                    .clickable {
                        navController.navigate("fullScreenAlert/${alert.id}")
                    }
            ) {
                Column(
                    modifier = Modifier.padding(6.dp)
                ) {
                    Row(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalArrangement = Arrangement.SpaceBetween
                    ) {
                        Text(
                            text = alert.title,
                            fontSize = 20.sp,
                            color = MaterialTheme.colorScheme.secondary,
                            lineHeight = 21.sp
                        )
                        val dateFormat = SimpleDateFormat("yyyy-MM-dd", Locale.UK)
                        Text(
                            text = dateFormat.format(alert.date),
                            fontSize = 14.sp,
                            color = MaterialTheme.colorScheme.secondary,
                            lineHeight = 21.sp
                        )
                    }

                    Text(
                        text = alert.description,
                        modifier = Modifier
                            .fillMaxWidth()
                            .padding(top = 10.dp),
                        fontSize = 17.sp,
                        color = MaterialTheme.colorScheme.surfaceBright,
                        textAlign = TextAlign.Start,
                        overflow = TextOverflow.Ellipsis,
                        lineHeight = 20.sp,
                        maxLines = 4
                    )
                }
            }
        }
    }
}

@Composable
fun FullScreenAlertScreen(navController: NavController, alertId: Long, context: Context) {
    val alerts = getAlertsFromSharedPreferences(LocalContext.current)
    val alert = alerts.firstOrNull { it.id == alertId }

    if (alert != null) {
        Column(
            modifier = Modifier
            .fillMaxSize()
            .padding(16.dp)
        ) {
            Row(
                horizontalArrangement = Arrangement.Absolute.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically,
                modifier = Modifier.fillMaxWidth()
            ){
                TextButton(
                    onClick = { navController.navigate("alerts") }
                ) {
                    Text(
                        "< Back",
                        color = MaterialTheme.colorScheme.surfaceBright,
                        fontSize = 18.sp,
                        modifier = Modifier.padding(0.dp)
                    )
                }
                TextButton(
                    onClick = {
                        navController.navigate("alerts")
                        saveAlertsToSharedPreferences(context, alerts.filter { it.id != alertId })
                    }
                ) {
                    Text(
                        "Delete",
                        color = MaterialTheme.colorScheme.error,
                        fontSize = 18.sp,
                        modifier = Modifier.padding(0.dp)
                    )
                }
            }
            Text(
                alert.title,
                fontSize = 40.sp,
                color = MaterialTheme.colorScheme.secondary,
                lineHeight = 42.sp
            )

            Spacer(modifier = Modifier.height(4.dp))

            Text(
                text = alert.date.toString(),
                fontSize = 18.sp,
                color = MaterialTheme.colorScheme.secondary,
                lineHeight = 21.sp
            )

            LazyColumn {
                item() {
                    Column() {
                        Spacer(modifier = Modifier.height(8.dp))
                        Text(
                            alert.description,
                            modifier = Modifier
                                .fillMaxWidth()
                                .padding(top = 10.dp),
                            fontSize = 25.sp,
                            color = MaterialTheme.colorScheme.surfaceBright,
                            textAlign = TextAlign.Start,
                            lineHeight = 27.sp,
                        )
                    }
                }
            }
        }
    } else {
        Text("Alert not found!")
    }
}