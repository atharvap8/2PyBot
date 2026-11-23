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
import `in`.twopybot.app.ui.isCompactHeight
import `in`.twopybot.app.ui.theme.*

/** Bound the tool pane; give the remaining width to the active workspace. */
@Composable
fun AdaptiveWorkspace(modifier:Modifier=Modifier,portraitRatio:Float=16f/10f,primary:@Composable (Modifier)->Unit,tools:@Composable ColumnScope.()->Unit) {
    val compact=isCompactHeight()
    BoxWithConstraints(modifier.fillMaxSize()) {
        // Sideways phone: a wider, independently scrolling tool column next to a
        // full-height primary view; tighter gaps so nothing is wasted vertically.
        val toolWidth=(maxWidth*(if(compact) .34f else .29f)).coerceIn(230.dp,340.dp)
        val gap=if(compact) 8.dp else 12.dp
        if(maxWidth>=520.dp) Row(Modifier.fillMaxSize().padding(gap),horizontalArrangement=Arrangement.spacedBy(gap)) {
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
        if(maxWidth>=560.dp) Row(Modifier.fillMaxSize().padding(horizontal=12.dp),horizontalArrangement=Arrangement.spacedBy(12.dp)) {
            Column(Modifier.weight(1f).fillMaxHeight().verticalScroll(rememberScrollState()),content=first)
            Column(Modifier.weight(1.25f).fillMaxHeight().verticalScroll(rememberScrollState()),content=second)
        } else Column(Modifier.fillMaxSize().verticalScroll(rememberScrollState()).padding(horizontal=14.dp)) {
            first();second();Spacer(Modifier.height(14.dp))
        }
    }
}

@Composable
fun WorkspaceTitle(title:String,subtitle:String?=null,modifier:Modifier=Modifier) {
    val compact=isCompactHeight()
    Column(modifier.padding(top=if(compact) 6.dp else 14.dp,bottom=if(compact) 6.dp else 10.dp)) {
        Text(title,fontSize=if(compact) 17.sp else 21.sp,fontWeight=FontWeight.SemiBold,color=TextHi)
        if(subtitle!=null && !compact) Text(subtitle,fontSize=12.sp,color=TextDim,modifier=Modifier.padding(top=3.dp))
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
