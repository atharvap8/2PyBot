package `in`.twopybot.app.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import `in`.twopybot.app.ui.theme.*

/** Bound the tool pane; give the remaining width to the active workspace. */
@Composable
fun AdaptiveWorkspace(modifier:Modifier=Modifier,portraitRatio:Float=16f/10f,primary:@Composable (Modifier)->Unit,tools:@Composable ColumnScope.()->Unit) {
    BoxWithConstraints(modifier.fillMaxSize()) {
        val toolWidth=(maxWidth*.29f).coerceIn(200.dp,300.dp)
        if(maxWidth>=520.dp) Row(Modifier.fillMaxSize().padding(12.dp),horizontalArrangement=Arrangement.spacedBy(12.dp)) {
            primary(Modifier.weight(1f).fillMaxHeight())
            Column(Modifier.width(toolWidth).fillMaxHeight()
                .verticalScroll(rememberScrollState()),verticalArrangement=Arrangement.spacedBy(8.dp),content=tools)
        } else Column(Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(14.dp),verticalArrangement=Arrangement.spacedBy(12.dp)) {
            primary(Modifier.fillMaxWidth().aspectRatio(portraitRatio))
            tools()
            Spacer(Modifier.height(8.dp))
        }
    }
}

/** Two independently scrolling settings columns when the window has room. */
@Composable
fun WorkspaceColumns(first:@Composable ColumnScope.()->Unit,second:@Composable ColumnScope.()->Unit) {
    BoxWithConstraints(Modifier.fillMaxSize()) {
        if(maxWidth>=560.dp) Row(Modifier.fillMaxSize().padding(horizontal=14.dp),horizontalArrangement=Arrangement.spacedBy(14.dp)) {
            Column(Modifier.weight(1f).fillMaxHeight().verticalScroll(rememberScrollState()),content=first)
            Column(Modifier.weight(1.25f).fillMaxHeight().verticalScroll(rememberScrollState()),content=second)
        } else Column(Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(horizontal=14.dp)) {
            first();second();Spacer(Modifier.height(14.dp))
        }
    }
}

@Composable
fun WorkspaceTitle(title:String,subtitle:String?=null,modifier:Modifier=Modifier) {
    Column(modifier.padding(top=14.dp,bottom=10.dp)) {
        Text(title,fontSize=21.sp,fontWeight=FontWeight.SemiBold,color=TextHi)
        if(subtitle!=null) Text(subtitle,fontSize=12.sp,color=TextDim,modifier=Modifier.padding(top=3.dp))
    }
}

@Composable
fun InstrumentRow(label:String,value:String,accent:Color=TextHi) {
    Row(Modifier.fillMaxWidth().heightIn(min=37.dp),verticalAlignment=Alignment.CenterVertically,
        horizontalArrangement=Arrangement.spacedBy(8.dp)) {
        Text(label,fontSize=12.sp,color=TextDim,modifier=Modifier.weight(1f),maxLines=1,overflow=TextOverflow.Ellipsis)
        Text(value,style=Mono.copy(fontSize=13.sp),color=accent,maxLines=1)
    }
}
