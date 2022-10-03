package com.example.taxi.ui.mypage.update_provider

import android.app.Dialog
import android.content.Context
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.DialogFragment
import com.example.taxi.R
import com.example.taxi.databinding.DlgUpdateProviderBinding

class UpdateProviderDialogFragment(val title : String) : DialogFragment() {
    private var _binding: DlgUpdateProviderBinding? = null
    private val binding get() = _binding!!

    private lateinit var listener : UpdateProviderDialogOKClickedListener


    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?, savedInstanceState: Bundle?): View? {
        _binding = DlgUpdateProviderBinding.inflate(inflater, container, false)
        val view = binding.root

        if(title == "carName"){
            binding.imageUpdateCarInfo.setImageResource(R.drawable.ic_local_taxi)
            binding.textDlgUpdateCarTitle.text = "차량 이름"
            binding.edittextDlgCarInfoInput.hint = "차량 이름을 입력해주세요"
        }else if(title == "carNumber"){
            binding.imageUpdateCarInfo.setImageResource(R.drawable.ic_tag)
            binding.textDlgUpdateCarTitle.text = "차량 번호"
            binding.edittextDlgCarInfoInput.hint = "차량 번호를 입력해주세요"
        }

        //ok 버튼 동작
        binding.buttonDlgUpdateCarInfo.setOnClickListener {
            listener.onOKClicked(binding.edittextDlgCarInfoInput.text.toString())
            dismiss()
        }

        return view
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }

    interface UpdateProviderDialogOKClickedListener {
        fun onOKClicked(content : String)
    }

    fun setOnOKClickedListener(listener: (String) -> Unit) {
        this.listener = object: UpdateProviderDialogOKClickedListener {
            override fun onOKClicked(content: String) {
                listener(content)
            }
        }
    }
}