package com.example.taxi.ui.login

import android.content.Context
import android.content.pm.PackageInfo
import android.content.pm.PackageManager
import android.os.Build
import android.util.Base64
import android.util.Log
import androidx.annotation.RequiresApi
import androidx.browser.customtabs.CustomTabsClient.getPackageName
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.databinding.FragmentLoginBinding
import com.example.taxi.di.ApplicationClass
import com.google.android.gms.common.wrappers.Wrappers.packageManager
import dagger.hilt.android.AndroidEntryPoint
import java.security.MessageDigest
import java.security.NoSuchAlgorithmException


@AndroidEntryPoint
class LoginFragment : BaseFragment<FragmentLoginBinding>(R.layout.fragment_login) {
    val authViewModel: AuthViewModel by viewModels()

    override fun init() {
        setOnClickListeners()
    }

    private fun setOnClickListeners(){
        binding.textLoginSignup.setOnClickListener {
            findNavController().navigate(R.id.action_loginFragment_to_joinFragment)
        }
        binding.buttonLoginEmailLogin.setOnClickListener {
            findNavController().navigate(R.id.action_loginFragment_to_emailLoginFragment)
        }
    }

    override fun onStart() {
        super.onStart()
        authViewModel.getSession { user ->
            if (user != null){
                ApplicationClass.userId = user.userId
                ApplicationClass.prefs.name = user.name
                ApplicationClass.prefs.userSeq = user.userSeq
                ApplicationClass.prefs.useCount = user.useCount
                if(user.isEachProvider){
                    findNavController().navigate(R.id.action_loginFragment_to_providerHomeFragment)
                }else{
                    findNavController().navigate(R.id.action_loginFragment_to_userHomeFragment)
                }
            }
        }
    }

}