package com.example.sdpapp.ui

import android.content.Context
import android.os.Build
import androidx.annotation.RequiresApi
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.rememberScrollState
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
import androidx.compose.ui.Modifier
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
import java.time.Instant
import java.time.YearMonth

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

    val alerts = remember { getAlertsFromSharedPreferences(context) }

    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(MaterialTheme.colorScheme.background)
            .padding(horizontal = 20.dp)
            .verticalScroll(rememberScrollState())
    ) {
        TextButton(
            onClick = { navController.navigate("home") }
        ) {
            Text(
                "< back",
                color = MaterialTheme.colorScheme.surfaceBright,
                fontSize = 18.sp,
                modifier = Modifier.padding(0.dp)
            )
        }

        AlertList(alerts, navController)
    }
}

@Composable
fun AlertList(alerts: List<Alert>, navController: NavController) {
    for (alert in alerts) {
        OutlinedCard(
            colors = CardDefaults.cardColors(
                containerColor = MaterialTheme.colorScheme.onBackground,
            ),
            border = BorderStroke(2.dp, MaterialTheme.colorScheme.tertiary),
            modifier = Modifier
                .fillMaxWidth()
                .height(130.dp)
                .padding(bottom = 10.dp)
                .clickable {
                    navController.navigate("fullScreenAlert/${alert.id}")
                }
        ) {
            Column(
                modifier = Modifier.padding(12.dp)
            ) {
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.SpaceBetween
                ) {
                    Text(
                        text = alert.title,
                        fontSize = 20.sp,
                        color = MaterialTheme.colorScheme.secondary
                    )
                    Text(
                        text = alert.date.toString(),
                        fontSize = 14.sp,
                        color = MaterialTheme.colorScheme.primary
                    )
                }

                Spacer(modifier = Modifier.height(8.dp))

                Text(
                    text = alert.description,
                    modifier = Modifier.fillMaxWidth(),
                    fontSize = 16.sp,
                    color = MaterialTheme.colorScheme.secondary,
                    textAlign = TextAlign.Start,
                    maxLines = 2,
                    overflow = TextOverflow.Ellipsis
                )
            }
        }
    }
}

@Composable
fun FullScreenAlertScreen(alertId: Long) {
    val alerts = getAlertsFromSharedPreferences(LocalContext.current)

    val alert = alerts.firstOrNull { it.id == alertId }

    if (alert != null) {
        Column(modifier = Modifier.fillMaxSize().padding(16.dp)) {
            Text("Full Screen Alert: ${alert.title}")
            Spacer(modifier = Modifier.height(8.dp))
            Text("Description: ${alert.description}")
        }
    } else {
        Text("Alert not found!")
    }
}


@Composable
fun getAlertById(alertId: Long): Alert {
    val alerts = getAlertsFromSharedPreferences(LocalContext.current)
    return alerts.first { it.id == alertId }
}