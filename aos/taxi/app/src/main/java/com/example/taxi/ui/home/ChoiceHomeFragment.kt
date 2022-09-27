package com.example.taxi.ui.home

import androidx.navigation.fragment.findNavController
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.databinding.FragmentChoiceHomeBinding
import com.example.taxi.di.ApplicationClass

class ChoiceHomeFragment : BaseFragment<FragmentChoiceHomeBinding>(R.layout.fragment_choice_home) {



    override fun init() {
        setOnClickListener()
    }

    private fun setOnClickListener() {
        binding.button.setOnClickListener {
            findNavController().navigate(R.id.action_choiceHomeFragment_to_userHomeFragment)
        }
        binding.button2.setOnClickListener {
            ApplicationClass.prefs.providerId = ApplicationClass.userId
            findNavController().navigate(R.id.action_choiceHomeFragment_to_providerHomeFragment)
        }
    }


}