package com.example.taxi.ui.login.join

import androidx.fragment.app.viewModels
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.databinding.FragmentJoinBinding
import com.example.taxi.ui.login.AuthViewModel
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class JoinProviderFragment : BaseFragment<FragmentJoinBinding>(R.layout.fragment_join) {

    override fun init() {
        setOnClickListeners()
    }

    private fun setOnClickListeners(){

    }
}