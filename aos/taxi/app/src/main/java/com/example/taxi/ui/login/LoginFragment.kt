package com.example.taxi.ui.login

import android.content.ContentValues
import android.content.Intent
import android.util.Log
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.databinding.FragmentLoginBinding
import com.example.taxi.di.ApplicationClass
import com.example.taxi.utils.view.toast
import com.google.android.gms.auth.api.signin.GoogleSignIn
import com.google.android.gms.auth.api.signin.GoogleSignInClient
import com.google.android.gms.auth.api.signin.GoogleSignInOptions
import com.google.android.gms.common.api.ApiException
import com.google.firebase.auth.FirebaseAuth
import com.google.firebase.auth.GoogleAuthProvider
import com.google.firebase.auth.ktx.auth
import com.google.firebase.ktx.Firebase
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class LoginFragment : BaseFragment<FragmentLoginBinding>(R.layout.fragment_login) {
    val authViewModel: AuthViewModel by viewModels()
    private lateinit var googleSignInClient: GoogleSignInClient
    private lateinit var auth : FirebaseAuth

    override fun init() {
        auth = Firebase.auth

        // Configure Google Signin
        var gso = GoogleSignInOptions.Builder(GoogleSignInOptions.DEFAULT_SIGN_IN)
            .requestIdToken("428302724497-13de29pv4nuk02lues1e0cg5sn2oq3md.apps.googleusercontent.com")
            .requestEmail()
            .build()
        //TODO : Google Login 및 SNS Login 구현
        //googleSignInClient = GoogleSignIn.getClient(requireContext(), gso)

        setOnClickListeners()
    }

    private fun setOnClickListeners(){
        binding.imageLoginGoogle.setOnClickListener{
            googleSignIn()
//            findNavController().navigate(R.id.action_loginFragment_to_joinFragment)
        }
        binding.buttonLoginEmailLogin.setOnClickListener {
            findNavController().navigate(R.id.action_loginFragment_to_emailLoginFragment)
        }
        binding.textLoginSignup.setOnClickListener {
            findNavController().navigate(R.id.action_loginFragment_to_joinFragment)
        }
    }

    private fun googleSignIn(){
        val signInIntent = googleSignInClient.signInIntent
        startActivityForResult(signInIntent, RC_SIGN_IN)
    }

    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        super.onActivityResult(requestCode, resultCode, data)

        // Result returned from launching the Intent from GoogleSignInApi.getSignInIntent(...);
        if (requestCode == RC_SIGN_IN) {
            val task = GoogleSignIn.getSignedInAccountFromIntent(data)
            try {
                // Google Sign In was successful, authenticate with Firebase
                val account = task.getResult(ApiException::class.java)!!
                Log.d(ContentValues.TAG, "firebaseAuthWithGoogle:" + account.id)
                firebaseAuthWithGoogle(account.idToken!!)
            } catch (e: ApiException) {
                // Google Sign In failed, update UI appropriately
                Log.w(ContentValues.TAG, "Google sign in failed", e)
            }
        }
    }

    private fun firebaseAuthWithGoogle(idToken: String) {
        println("firebaseAuthWithGoogle")
        val credential = GoogleAuthProvider.getCredential(idToken, null)
        auth.signInWithCredential(credential)
            .addOnCompleteListener(requireActivity()) { task ->
                if (task.isSuccessful) {
                    // Sign in success, update UI with the signed-in user's information
                    Log.d(ContentValues.TAG, "signInWithCredential:success")
                    val user = auth.currentUser
//                    updateUI(user)
                    findNavController().navigate(R.id.action_loginFragment_to_joinFragment)
                } else {
                    // If sign in fails, display a message to the user.
                    Log.w(ContentValues.TAG, "signInWithCredential:failure", task.exception)
//                    updateUI(null)
                }
            }
    }

    override fun onStart() {
        super.onStart()
        authViewModel.getSession { user ->
            if (user != null){
                ApplicationClass.userId = user.userId
                ApplicationClass.prefs.name = user.name
                ApplicationClass.prefs.userSeq = user.userSeq
                ApplicationClass.prefs.tel = user.tel
                ApplicationClass.prefs.useCount = user.useCount

                if(user.isEachProvider){
                    findNavController().navigate(R.id.action_loginFragment_to_providerHomeFragment)
                }else{
                    findNavController().navigate(R.id.action_loginFragment_to_userHomeFragment)
                }
            }
        }
    }
    companion object{
        const val RC_SIGN_IN = 1001
        const val EXTRA_NAME = "EXTRA NAME"
    }
}