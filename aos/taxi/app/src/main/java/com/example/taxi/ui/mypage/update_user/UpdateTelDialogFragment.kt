package com.example.taxi.ui.mypage.update_user

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.DialogFragment
import com.example.taxi.databinding.DlgTelBinding
import com.example.taxi.utils.constant.isValidEmail
import com.example.taxi.utils.view.toast

class UpdateTelDialogFragment : DialogFragment() {
    private var _binding: DlgTelBinding? = null
    private val binding get() = _binding!!

    private lateinit var listener : TellDialogOKClickedListener


    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?, savedInstanceState: Bundle?): View? {
        _binding = DlgTelBinding.inflate(inflater, container, false)
        val view = binding.root

        //ok 버튼 동작
        binding.buttonDlgTelUpdate.setOnClickListener {
            if (validation()) {
                val tel =
                    binding.edittextTelInput1.text.toString() + "-" + binding.edittextTelInput2.text.toString() + "-" + binding.edittextTelInput3.text.toString()
                listener.onOKClicked(tel)
                dismiss()
            }
        }

        return view
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }

    interface TellDialogOKClickedListener {
        fun onOKClicked(content : String)
    }

    fun setOnOKClickedListener(listener: (String) -> Unit) {
        this.listener = object: TellDialogOKClickedListener {
            override fun onOKClicked(content: String) {
                listener(content)
            }
        }
    }

    private fun validation(): Boolean {
        var isValid = true

        if (binding.edittextTelInput1.text.isNullOrEmpty()){
            isValid = false
            toast("핸드폰번호를 입력해 주세요.")
        }

        if (binding.edittextTelInput2.text.isNullOrEmpty()){
            isValid = false
            toast("핸드폰번호를 입력해 주세요.")
        }

        if (binding.edittextTelInput3.text.isNullOrEmpty()){
            isValid = false
            toast("핸드폰번호를 입력해 주세요.")
        }

        return isValid
    }
}