package `in`.twopybot.app.ui.screens

import android.content.Intent
import android.net.Uri
import android.view.ViewGroup
import android.webkit.WebSettings
import android.webkit.WebResourceError
import android.webkit.WebResourceRequest
import android.webkit.WebView
import android.webkit.WebViewClient
import androidx.activity.compose.BackHandler
import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.Alignment
import androidx.compose.ui.draw.clipToBounds
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import androidx.compose.ui.viewinterop.AndroidView
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import androidx.lifecycle.compose.LocalLifecycleOwner
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.LifecycleEventObserver
import `in`.twopybot.app.ui.theme.*
import `in`.twopybot.app.vm.BotViewModel
import `in`.twopybot.app.BuildConfig

@Composable
fun CalibrationScreen(vm: BotViewModel, page: String = "/ui/calibration.html", backLabel: String = "Back to Camera", onBack: () -> Unit) {
    val host by vm.repo.host.collectAsStateWithLifecycle()
    val context=LocalContext.current
    val embedded=page=="/ui/autonomy.html"
    val url="http://$host:8080$page"+if(embedded) "?embed=1&ui=${BuildConfig.VERSION_CODE}" else ""
    var web by remember { mutableStateOf<WebView?>(null) }
    var loading by remember(url) { mutableStateOf(true) }
    var error by remember(url) { mutableStateOf<String?>(null) }
    BackHandler { onBack() }
    fun external(link: String) {
        runCatching { context.startActivity(Intent(Intent.ACTION_VIEW,Uri.parse(link))) }
            .onFailure { vm.toast("Open the calibration page in your browser to download files") }
    }
    val lifecycle=LocalLifecycleOwner.current.lifecycle
    // Observe the actual view instance. Lifecycle changes must never destroy
    // whichever newer view happens to be stored in the mutable web reference.
    DisposableEffect(lifecycle,web) {
        val view=web
        val observer=LifecycleEventObserver { _,event ->
            if(event==Lifecycle.Event.ON_PAUSE) {
                view?.evaluateJavascript("window.onSupervisorPause && window.onSupervisorPause()",null)
                view?.onPause()
            } else if(event==Lifecycle.Event.ON_RESUME) {
                view?.onResume();view?.evaluateJavascript("window.supervisionActive=true",null)
            }
        }
        lifecycle.addObserver(observer)
        onDispose {
            lifecycle.removeObserver(observer)
        }
    }
    Column(Modifier.fillMaxSize()) {
        Row(Modifier.fillMaxWidth().height(42.dp).padding(horizontal=8.dp),horizontalArrangement=Arrangement.SpaceBetween) {
            TextButton(onClick=onBack,contentPadding=PaddingValues(horizontal=8.dp,vertical=0.dp)) { Text(backLabel,style=MaterialTheme.typography.labelLarge) }
            Row {
                TextButton(onClick={error=null;loading=true;web?.reload()},contentPadding=PaddingValues(horizontal=8.dp,vertical=0.dp)) { Text("Reload",style=MaterialTheme.typography.labelLarge) }
                TextButton(onClick={ external(url) },contentPadding=PaddingValues(horizontal=8.dp,vertical=0.dp)) { Text("Browser",style=MaterialTheme.typography.labelLarge) }
            }
        }
        Box(Modifier.weight(1f).fillMaxWidth().clipToBounds()) {
        AndroidView(modifier=Modifier.fillMaxSize(),factory={ c ->
            WebView(c).apply {
                layoutParams=ViewGroup.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT,ViewGroup.LayoutParams.MATCH_PARENT)
                setBackgroundColor(0xff090c10.toInt())
                settings.javaScriptEnabled=true
                settings.domStorageEnabled=true
                settings.allowFileAccess=false
                settings.allowContentAccess=false
                settings.useWideViewPort=!embedded
                settings.loadWithOverviewMode=!embedded
                if(embedded)settings.cacheMode=WebSettings.LOAD_NO_CACHE
                webViewClient=object: WebViewClient() {
                    override fun onPageFinished(view:WebView,url:String) {
                        loading=false
                        view.evaluateJavascript("window.dispatchEvent(new Event('resize'))",null)
                    }
                    override fun onReceivedError(view:WebView,request:WebResourceRequest,failure:WebResourceError) {
                        if(request.isForMainFrame) {loading=false;error=failure.description.toString()}
                    }
                    override fun shouldOverrideUrlLoading(view: WebView, request: WebResourceRequest): Boolean {
                        val uri=request.url
                        if(uri.host!=Uri.parse(url).host || listOf(".pdf",".svg",".zip",".json").any { uri.path.orEmpty().endsWith(it) }) {
                            external(uri.toString()); return true
                        }
                        return false
                    }
                }
                setDownloadListener { link,_,_,_,_ -> external(link) }
                web=this
                loadUrl(url)
            }
        },update={
            if(it.url?.substringBefore('?')!=url.substringBefore('?')) {loading=true;error=null;it.loadUrl(url)}
            it.requestLayout()
        },onReset=null,onRelease={view ->
            view.evaluateJavascript("window.onSupervisorPause && window.onSupervisorPause()",null)
            view.stopLoading();view.onPause();view.destroy()
        })
        if(loading) LinearProgressIndicator(modifier=Modifier.fillMaxWidth().align(Alignment.TopCenter))
        error?.let { description ->
            Column(Modifier.align(Alignment.Center).padding(24.dp),horizontalAlignment=Alignment.CenterHorizontally) {
                Text("Page could not load",style=MaterialTheme.typography.titleMedium)
                Text(description,color=TextDim,modifier=Modifier.padding(vertical=12.dp))
                Button(onClick={error=null;loading=true;web?.loadUrl(url)}) {Text("Reload page")}
            }
        }
        }
    }
}
