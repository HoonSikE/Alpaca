package com.example.taxi.ui.login.join

import android.app.Dialog
import android.content.Context
import android.graphics.Color
import android.graphics.drawable.ColorDrawable
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.view.Window
import androidx.appcompat.app.AppCompatActivity
import androidx.fragment.app.DialogFragment
import com.example.taxi.R
import com.example.taxi.databinding.DlgAddressBinding
import com.example.taxi.databinding.DlgPhoneAuthBinding
import com.example.taxi.utils.view.toast

class PhoneAuthDialogFragment() : DialogFragment() {
    private var _binding: DlgPhoneAuthBinding? = null
    private val binding get() = _binding!!

    private lateinit var listener : PhoneAuthDialogOKClickedListener


    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?, savedInstanceState: Bundle?): View? {
        _binding = DlgPhoneAuthBinding.inflate(inflater, container, false)
        dialog?.window?.setBackgroundDrawable(ColorDrawable(Color.TRANSPARENT))
        dialog?.window?.requestFeature(Window.FEATURE_NO_TITLE)
        val view = binding.root

        //ok 버튼 동작
        binding.buttonDlgAddressUpdate.setOnClickListener {
            if(binding.edittextDlgPhoneAuthInput.text.toString() == ""){
                toast("인증번호를 입력해주세요.")
            }else{
                listener.onOKClicked(binding.edittextDlgPhoneAuthInput.text.toString())
                dismiss()
            }
        }

        return view
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }

    interface PhoneAuthDialogOKClickedListener {
        fun onOKClicked(content : String)
    }

    fun setOnOKClickedListener(listener: (String) -> Unit) {
        this.listener = object: PhoneAuthDialogOKClickedListener {
            override fun onOKClicked(content: String) {
                listener(content)
            }
        }
    }
}