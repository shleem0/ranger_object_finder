package com.example.sdpapp.ui

import android.widget.Toast
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.ArrowDropDown
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.navigation.NavController

@Composable
fun SearchScreen(navController: NavController) {
    var expanded by remember { mutableStateOf(false) }
    var selectedOption by remember { mutableStateOf("Select an Item") }
    val context = LocalContext.current

    val options = getItemNames(context)

    Column(
        modifier = Modifier
            .fillMaxSize()
            .background(MaterialTheme.colorScheme.background)
            .padding(horizontal = 10.dp)
    ) {
        TextButton(
            onClick = { navController.navigate("home") }
        ) {
            Text(
                "< back",
                color = MaterialTheme.colorScheme.surfaceBright,
                fontSize = 18.sp
            )
        }

        Text(
            text = "Find Item",
            style = MaterialTheme.typography.bodyLarge,
            color = MaterialTheme.colorScheme.surfaceBright
        )

        Text(
            text = "Choose Item",
            color = MaterialTheme.colorScheme.surfaceBright,
            fontSize = 30.sp,
            fontWeight = FontWeight.Bold,
            modifier = Modifier.padding(vertical = 10.dp)
        )

        Column(modifier = Modifier.fillMaxWidth()) {
            OutlinedButton(
                onClick = { expanded = true },
                modifier = Modifier
                    .fillMaxWidth()
                    .height(60.dp)
            ) {
                Text(
                    text = selectedOption,
                    fontSize = 22.sp,
                    color = MaterialTheme.colorScheme.secondary
                )
                Icon(Icons.Filled.ArrowDropDown, contentDescription = "Dropdown Arrow")
            }

            DropdownMenu(
                expanded = expanded,
                onDismissRequest = { expanded = false }
            ) {
                options.forEach { option ->
                    val formattedOption = option.replaceFirstChar { it.uppercase() }
                    DropdownMenuItem(
                        text = {
                            Text(
                                formattedOption,
                                fontSize = 25.sp
                            )
                        },
                        onClick = {
                            selectedOption = formattedOption
                            expanded = false
                        }
                    )
                }
            }
        }

        LazyColumn(
        ) {
            item {
                Text(
                    text = "Remember:",
                    color = MaterialTheme.colorScheme.surfaceBright,
                    fontSize = 30.sp,
                    fontWeight = FontWeight.Bold,
                    modifier = Modifier.padding(vertical = 10.dp)
                )
            }
            items(
                listOf(
                    " - The robot can take up to 7 minutes to find an item.",
                    " - The robot will only be able to find the item in an enclosed area.",
                    " - The robot will grab the item, so it is not suitable for fragile objects."
                )
            ) { text ->
                Text(
                    text = text,
                    color = MaterialTheme.colorScheme.secondary,
                    fontSize = 18.sp,
                    lineHeight = 22.sp
                )
                Spacer(Modifier.padding(bottom = 5.dp))
            }
        }
    }
    Box(
        modifier = Modifier
            .fillMaxSize().padding(16.dp),
        contentAlignment = Alignment.BottomEnd
    ) {
        Box(
            contentAlignment = Alignment.TopEnd
        ) {
            Button(
                onClick = { navController.navigate("home")
                    Toast.makeText(context,
                    "Ranger will alert you when the item is found.",
                    Toast.LENGTH_SHORT).show()
                          },
                modifier = Modifier
                    .height(70.dp)
                    .fillMaxWidth()
                    .border(
                        BorderStroke(12.dp, MaterialTheme.colorScheme.secondary),
                        shape = RoundedCornerShape(16.dp)
                    ),
                colors = ButtonDefaults.buttonColors(
                    containerColor = MaterialTheme.colorScheme.secondary,
                    contentColor = MaterialTheme.colorScheme.onBackground
                )
            ) {
                Text(
                    "Find Item",
                    fontSize = 34.sp,
                    color = MaterialTheme.colorScheme.onBackground
                )
            }
        }
    }
}