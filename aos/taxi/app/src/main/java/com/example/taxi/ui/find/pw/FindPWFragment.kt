package com.example.taxi.ui.find.pw

import androidx.navigation.fragment.findNavController
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.databinding.FragmentFindPwBinding
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class FindPWFragment : BaseFragment<FragmentFindPwBinding>(R.layout.fragment_find_pw) {
    override fun init() {
        setOnClickListeners()
//        observerData()
    }

    private fun setOnClickListeners() {
        binding.buttonFindPw.setOnClickListener{
            findNavController().navigate(R.id.action_findPWFragment_to_loginFragment)
        }
    }
}