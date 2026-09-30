package `in`.twopybot.app.vm

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import `in`.twopybot.app.BuildConfig
import `in`.twopybot.app.data.BotRepository
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.launch

class BotViewModel(app: Application) : AndroidViewModel(app) {

    private val prefs = app.getSharedPreferences("2pybot", 0)
    val repo = BotRepository(prefs.getString("host", BuildConfig.DEFAULT_HOST)!!)

    val snack = MutableStateFlow<String?>(null)
    val trail = MutableStateFlow<List<Pair<Float, Float>>>(emptyList())

    init {
        repo.start()
        viewModelScope.launch { repo.refreshStatus(); repo.refreshParams() }
    }

    fun setHost(h: String) {
        prefs.edit().putString("host", h).apply()
        repo.setHost(h)
        toast("robot address set to $h")
    }

    fun toast(m: String) { snack.value = m }
    fun clearToast() { snack.value = null }

    fun setParam(key: String, v: Float) = viewModelScope.launch {
        repo.setParam(key, v).onFailure { toast(it.message ?: "rejected by robot") }
    }

    fun paramAction(a: String) = viewModelScope.launch {
        repo.paramAction(a).fold(
            { toast(if (a == "save") "saved to robot — survives reboot" else "$a done") },
            { toast(it.message ?: "failed") })
    }

    fun command(line: String) = viewModelScope.launch {
        repo.command(line).fold({ toast("sent $line") }, { toast(it.message ?: "refused") })
    }

    fun autoTune(routine: String) = viewModelScope.launch {
        repo.autoTune(routine).fold(
            { toast("auto-tune: $routine") },
            { toast(it.message ?: "refused — is AT, whitelisted on the console?") })
    }

    fun estop() = viewModelScope.launch {
        repo.estop().fold({ toast("E-STOP sent") }, { toast("E-STOP FAILED — ${it.message}") })
    }

    fun refreshTrail() = viewModelScope.launch { trail.value = repo.trail() }
    fun resetPose() = viewModelScope.launch { repo.resetPose(); trail.value = emptyList() }
}
