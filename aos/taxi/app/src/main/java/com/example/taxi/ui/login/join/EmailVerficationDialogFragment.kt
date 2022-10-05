package com.example.taxi.ui.login.join

import android.graphics.Color
import android.graphics.drawable.ColorDrawable
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.view.Window
import androidx.fragment.app.DialogFragment
import com.example.taxi.databinding.DlgEmailVerificationBinding
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class EmailVerficationDialogFragment(val email: String) : DialogFragment() {
    private var _binding: DlgEmailVerificationBinding? = null
    private val binding get() = _binding!!

    private lateinit var listener : EmailVerficationDialogOKClickedListener

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?, savedInstanceState: Bundle?): View? {
        _binding = DlgEmailVerificationBinding.inflate(inflater, container, false)
        dialog?.window?.setBackgroundDrawable(ColorDrawable(Color.TRANSPARENT))
        dialog?.window?.requestFeature(Window.FEATURE_NO_TITLE)
        val view = binding.root

        binding.textDlgEmailVerfication1.text = email

        //ok 버튼 동작
        binding.buttonDlgEmailVerfication.setOnClickListener {
            listener.onOKClicked(binding.buttonDlgEmailVerfication.text.toString())
            dismiss()
        }
        return view
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }

    interface EmailVerficationDialogOKClickedListener {
        fun onOKClicked(content : String)
    }

    fun setOnOKClickedListener(listener: (String) -> Unit) {
        this.listener = object: EmailVerficationDialogOKClickedListener {
            override fun onOKClicked(content: String) {
                listener(content)
            }
        }
    }
}