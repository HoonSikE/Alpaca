package com.example.taxi.ui.home.provider

import android.graphics.Color
import android.graphics.drawable.ColorDrawable
import android.os.Bundle
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.view.Window
import android.widget.TimePicker
import androidx.fragment.app.DialogFragment
import androidx.fragment.app.viewModels
import com.example.taxi.data.dto.provider.Provider
import com.example.taxi.databinding.DlgProviderBinding
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint
import kr.co.bootpay.android.constants.BootpayConstant.confirm

@AndroidEntryPoint
class ProviderDialogFragment(val confirm : () -> Unit) : DialogFragment() {

    private var _binding: DlgProviderBinding? = null
    private val binding get() = _binding!!
    private val providerViewModel : ProviderViewModel by viewModels()
    private lateinit var provider : Provider

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {
        _binding = DlgProviderBinding.inflate(inflater, container, false)
        dialog?.window?.setBackgroundDrawable(ColorDrawable(Color.TRANSPARENT))
        dialog?.window?.requestFeature(Window.FEATURE_NO_TITLE)
        return binding.root
    }

    private var str = ""

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        observerData()
        setOnClickListeners()
    }

    private fun observerData() {
        providerViewModel.provider.observe(viewLifecycleOwner) { state ->
            when (state) {
                is UiState.Loading -> {
                    //binding.progressBar.show()
                }
                is UiState.Failure -> {
                    //binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                }
                is UiState.Success -> {
                    provider = state.data
                    provider.car!!.deadLine = str
                    providerViewModel.updateProvider(provider.car!!)
                }
            }
        }
        providerViewModel.providerCar.observe(viewLifecycleOwner) { state ->
            when (state) {
                is UiState.Loading -> {
                    //binding.progressBar.show()
                }
                is UiState.Failure -> {
                    //binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                }
                is UiState.Success -> {
                    Log.d("UiState.Success", "providerCar clear")
                    confirm()
                    dismiss()
                }
            }
        }
    }

    private fun setOnClickListeners() {
        binding.buttonDlgProviderProvider.setOnClickListener { providerViewModel.getProvider() }
        binding.buttonDlgProviderCancel.setOnClickListener { dismiss() }
        binding.buttonDlgProviderTimePicker.setOnTimeChangedListener{ timePicker, hour, minute -> // 오전 / 오후 를 확인하기 위한 if 문
            var hour = hour
            var minuteString = ""
            if (hour > 12) {
                hour -= 12
                if(minute < 10){
                    minuteString = "0$minute"
                    str = "PM $hour:$minuteString"
                }else{
                    str = "PM $hour:$minute"
                }
            } else {
                if(minute < 10){
                    minuteString = "0$minute"
                    str = "AM $hour:$minuteString"
                }else{
                    str = "AM $hour:$minute"
                }
            }
        }
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }

}