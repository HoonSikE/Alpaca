package com.example.taxi.ui.mypage.with_drawal

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.DialogFragment
import com.example.taxi.databinding.DlgWithDrawalBinding
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class UserWithDrawalDialogFragment() : DialogFragment() {
    private var _binding: DlgWithDrawalBinding? = null
    private val binding get() = _binding!!

    private lateinit var listener : WithDrawalDialogOKClickedListener

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?, savedInstanceState: Bundle?): View? {
        _binding = DlgWithDrawalBinding.inflate(inflater, container, false)
        val view = binding.root

        //ok 버튼 동작
        binding.buttonDlgUserWithDrawal.setOnClickListener {
            listener.onOKClicked(binding.buttonDlgUserWithDrawal.text.toString())
            dismiss()
        }
        return view
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }

    interface WithDrawalDialogOKClickedListener {
        fun onOKClicked(content : String)
    }

    fun setOnOKClickedListener(listener: (String) -> Unit) {
        this.listener = object: WithDrawalDialogOKClickedListener {
            override fun onOKClicked(content: String) {
                listener(content)
            }
        }
    }
}