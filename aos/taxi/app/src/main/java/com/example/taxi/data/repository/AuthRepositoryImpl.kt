package com.example.taxi.data.repository

import android.content.Context.MODE_PRIVATE
import android.content.SharedPreferences
import android.util.Log
import androidx.core.net.toUri
import androidx.fragment.app.FragmentActivity
import androidx.navigation.fragment.findNavController
import com.example.taxi.R
import com.example.taxi.data.dto.user.User
import com.example.taxi.di.ApplicationClass
import com.example.taxi.ui.login.join.JoinFragment
import com.example.taxi.utils.constant.FireStoreCollection
import com.example.taxi.utils.constant.SharedPrefConstants
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.view.toast
import com.facebook.AccessToken
import com.google.android.gms.tasks.OnFailureListener
import com.google.android.gms.tasks.OnSuccessListener
import com.google.firebase.FirebaseException
import com.google.firebase.FirebaseTooManyRequestsException
import com.google.firebase.auth.*
import com.google.firebase.auth.ktx.auth
import com.google.firebase.firestore.FirebaseFirestore
import com.google.firebase.storage.FirebaseStorage
import com.google.gson.Gson
import java.util.concurrent.TimeUnit


class AuthRepositoryImpl(
    val auth: FirebaseAuth,
    val database: FirebaseFirestore,
    val appPreferences: SharedPreferences,
    val gson: Gson
) : AuthRepository {
    override fun getCurrentUser(result: (FirebaseUser) -> Unit){
        val currentUser = auth.currentUser
        if (currentUser != null) {
            result.invoke(currentUser)
        } else {

        }
    }

    override fun registerUser(
        email: String,
        password: String,
        user: User,
        result: (UiState<String>) -> Unit
    ) {
        auth.createUserWithEmailAndPassword(email,password)
            .addOnCompleteListener {
                Log.d("registerUser : ", it.toString())
                if (it.isSuccessful){
                    user.userSeq = it.result.user?.uid ?: ""
                    updateUserInfo(user) { state ->
                        when(state){
                            is UiState.Success -> {
                                storeSession(id = it.result.user?.uid ?: "") {
                                    if (it == null){
                                        result.invoke(UiState.Failure("User register successfully but session failed to store"))
                                    }else{
                                        result.invoke(
                                            UiState.Success("User register successfully!")
                                        )
                                    }
                                }
                            }
                            is UiState.Failure -> {
                                result.invoke(UiState.Failure(state.error))
                            }
                            else -> {}
                        }
                    }
                }else{
                    try {
                        throw it.exception ?: java.lang.Exception("Invalid authentication")
                    } catch (e: FirebaseAuthWeakPasswordException) {
                        result.invoke(UiState.Failure("Authentication failed, Password should be at least 6 characters"))
                    } catch (e: FirebaseAuthInvalidCredentialsException) {
                        result.invoke(UiState.Failure("Authentication failed, Invalid email entered"))
                    } catch (e: FirebaseAuthUserCollisionException) {
                        result.invoke(UiState.Failure("Authentication failed, Email already registered."))
                    } catch (e: Exception) {
                        result.invoke(UiState.Failure(e.message))
                    }
                }
            }
            .addOnFailureListener {
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
            }
    }

    override fun snsRegister(user: User, result: (UiState<String>) -> Unit){
        user.userSeq = ApplicationClass.prefs.userSeq.toString()

        updateUserInfo(user) { state ->
            when (state) {
                is UiState.Success -> {
                    storeSession(id = user.userSeq ?: "") {
                        if (it == null) {
                            result.invoke(UiState.Failure("User register successfully but session failed to store"))
                        } else {
                            result.invoke(
                                UiState.Success("User register successfully!")
                            )
                        }
                    }
                }
                is UiState.Failure -> {
                    result.invoke(UiState.Failure(state.error))
                }
                else -> {}
            }
        }
    }

    override fun phoneAuth(phoneNumber: String, activity: FragmentActivity, result: (String) -> Unit){
        auth.setLanguageCode("ko-KR")

        println("option")
        println("phoneNumber : " + phoneNumber)
        val options = PhoneAuthOptions.newBuilder(auth)
            .setPhoneNumber(phoneNumber)
            .setTimeout(60L, TimeUnit.SECONDS)
            .setActivity(activity)
            .setCallbacks(object : PhoneAuthProvider.OnVerificationStateChangedCallbacks() {
                // 번호인증 혹은 기타 다른 인증(구글로그인, 이메일로그인 등) 끝난 상태
                override fun onVerificationCompleted(credential: PhoneAuthCredential) {
                    println("onVerificationCompleted")
                }

                // 번호인증 실패 상태
                override fun onVerificationFailed(e: FirebaseException) {
                    println("onVerificationFailed")
                    if (e is FirebaseAuthInvalidCredentialsException) {
                        // Invalid request
                    } else if (e is FirebaseTooManyRequestsException) {
                        // The SMS quota for the project has been exceeded
                    }
                }

                // 전화번호는 확인 되었으나 인증코드를 입력해야 하는 상태
                override fun onCodeSent(
                    verificationId: String,
                    token: PhoneAuthProvider.ForceResendingToken
                ) {
                    result.invoke(verificationId)
                }
            })
            .build()

        PhoneAuthProvider.verifyPhoneNumber(options)
    }

    override fun ckeckPhoneAuth(verificationId: String, code: String, result: (UiState<String>) -> Unit){
        val credential = PhoneAuthProvider.getCredential(verificationId!!, code)

        println("ckeckPhoneAuth : " + verificationId + " " + code)


        auth.signInWithCredential(credential)
            .addOnCompleteListener() { task ->
                if (task.isSuccessful) {
                    auth.signOut()
                    result.invoke(UiState.Success("User has been ckecked PhoneAuth successfully"))
                } else {
                    result.invoke(UiState.Failure("User has been ckecked PhoneAuth Failure"))
                }
            }
    }

    override fun updateUserInfo(user: User, result: (UiState<String>) -> Unit) {
        if(user.profileImage != "") {
            var storage = FirebaseStorage.getInstance()
            var imgFileName = user.userSeq + ".png"

            storage.getReference().child("user_profiles").child(imgFileName)
                .putFile(user.profileImage.toUri())//어디에 업로드할지 지정
                .addOnSuccessListener { taskSnapshot ->
                    taskSnapshot.metadata?.reference?.downloadUrl?.addOnSuccessListener { it ->
                        ApplicationClass.prefs.profileImage = it.toString()
                        user.profileImage = it.toString()
                        // firestore 업데이트
                        database.collection(FireStoreCollection.USER)
                            .document(user.userSeq)
                            .set(user)
                            .addOnSuccessListener { task ->
                                result.invoke(
                                    UiState.Success("User has been update successfully")
                                )
                            }.addOnFailureListener { task ->
                                result.invoke(
                                    UiState.Failure(
                                        task.localizedMessage
                                    )
                                )
                            }
                    }
                }.addOnFailureListener {
                    Log.d("addImageUpLoad", "Image has been uploaded fail")
                }
        }else{
            database.collection(FireStoreCollection.USER)
                .document(user.userSeq)
                .set(user)
                .addOnSuccessListener { task ->
                    result.invoke(
                        UiState.Success("User has been update successfully")
                    )
                }.addOnFailureListener { task ->
                    result.invoke(
                        UiState.Failure(
                            task.localizedMessage
                        )
                    )
                }
        }
    }

    override fun loginUser(email: String, password: String, result: (UiState<String>) -> Unit) {
        auth.signInWithEmailAndPassword(email,password)
            .addOnCompleteListener { task ->
                if (task.isSuccessful) {
                    storeSession(id = task.result.user?.uid ?: ""){
                        if (it == null){
                            result.invoke(UiState.Failure("Failed to store local session"))
                        }else{
                            result.invoke(UiState.Success("Login successfully!"))
                        }
                    }
                }
            }.addOnFailureListener {
                result.invoke(UiState.Failure("Authentication failed, Check email and password"))
            }
    }

    override fun googleLogin(idToken: String, result: (UiState<String>) -> Unit){
        val credential = GoogleAuthProvider.getCredential(idToken, null)

        auth.signInWithCredential(credential)
            .addOnCompleteListener { task ->
                if (task.isSuccessful) {
                    storeSession(id = idToken ?: ""){
                        if (it == null){
                            result.invoke(UiState.Success("null"))
                        }else{
                            result.invoke(UiState.Success("Login successfully!"))
                        }
                    }
                } else {
                    // If sign in fails, display a message to the user.
                    result.invoke(UiState.Success("Failed to store local session"))
                }
            }
            .addOnFailureListener {
                result.invoke(UiState.Failure("Authentication failed, Check google email"))
            }
    }

    override fun facebookLogin(accessToken: AccessToken, result: (UiState<String>) -> Unit){
        // AccessToken 으로 Facebook 인증
        val credential = FacebookAuthProvider.getCredential(accessToken?.token!!)

        // 성공 시 Firebase 에 유저 정보 보내기 (로그인)
        auth?.signInWithCredential(credential)
            ?.addOnCompleteListener{ task ->
                if (task.isSuccessful) {
                    storeSession(id = task.result.user?.uid ?: ""){
                        ApplicationClass.prefs.userSeq = task.result.user?.uid ?: ""
                        println("task.result.user?.uid : " + task.result.user?.uid)
                        if (it == null){
                            result.invoke(UiState.Success("Failed to facebook login"))
                        }else{
                            result.invoke(UiState.Success("Facebook login successfully!"))
                        }
                    }
                } else {
                    // If sign in fails, display a message to the user.
                    result.invoke(UiState.Success("Failed to facebook login"))
                }
            }
    }

    override fun githubLogin(activity: FragmentActivity, result: (UiState<String>) -> Unit){
        // 빌더로 OAuthProvider의 인스턴스를 생성
        val githubProvider = OAuthProvider.newBuilder("github.com")
        // 선택사항: OAuth 요청과 함께 전송하고자 하는 커스텀 OAuth 매개변수를 추가로 지정합니다.
        auth.startActivityForSignInWithProvider(activity, githubProvider.build())
            .addOnSuccessListener { authResult ->
                    auth.signInWithCredential(authResult.credential!!)
                        .addOnCompleteListener(activity) { task ->
                            if (task.isSuccessful) {
                                storeSession(id = task.result.user?.uid ?: ""){
                                    ApplicationClass.prefs.userSeq = task.result.user?.uid ?: ""
                                    println("task.result.user?.uid : " + task.result.user?.uid)
                                    if (it == null){
                                        result.invoke(UiState.Success("Failed to github login"))
                                    }else{
                                        result.invoke(UiState.Success("Github login successfully!"))
                                    }
                                }
                            } else {
                                // If sign in fails, display a message to the user.
                                result.invoke(UiState.Success("Failed to github login"))
                            }
                        }
            }.addOnFailureListener{
                result.invoke(UiState.Failure("Authentication failed, Check github email"))
            }

    }

    override fun reauthPassword(existingPassword: String, result: (UiState<String>) -> Unit){
        val credential = EmailAuthProvider
            .getCredential(ApplicationClass.userId, existingPassword)

        auth.currentUser!!.reauthenticate(credential)
            .addOnCompleteListener { task ->
                if(task.isSuccessful) {
                    result.invoke(UiState.Success("User re-authenticated."))
                }else{
                    result.invoke(UiState.Failure("Failed to store local session"))
                }
            }.addOnFailureListener{
                result.invoke(UiState.Failure("Authentication failed, Check password"))
            }
    }

    override fun updatePassword(newPassword: String, result: (UiState<String>) -> Unit) {
        auth.currentUser!!.updatePassword(newPassword)
            .addOnCompleteListener { task ->
                if (task.isSuccessful) {
                    result.invoke(UiState.Success("User password updated."))
                }
            }
    }

    override fun forgotPassword(email: String, result: (UiState<String>) -> Unit) {
        auth.setLanguageCode("ko-KR")
        auth.sendPasswordResetEmail(email)
            .addOnCompleteListener { task ->
                if (task.isSuccessful) {
                    result.invoke(UiState.Success("Email has been sent"))

                } else {
                    result.invoke(UiState.Failure(task.exception?.message))
                }
            }.addOnFailureListener {
                result.invoke(UiState.Failure("Authentication failed, Check email"))
            }
    }

    override fun logout(result: () -> Unit) {
        auth.signOut()
        appPreferences.edit().putString(SharedPrefConstants.USER_SESSION,null).apply()
        result.invoke()
    }

    override fun storeSession(id: String, result: (User?) -> Unit) {
        database.collection(FireStoreCollection.USER).document(id)
            .get()
            .addOnCompleteListener {
                if (it.isSuccessful){
                    val user = it.result.toObject(User::class.java)
                    appPreferences.edit().putString(SharedPrefConstants.USER_SESSION,gson.toJson(user)).apply()
                    result.invoke(user)
                }else{
                    result.invoke(null)
                }
            }
            .addOnFailureListener {
                result.invoke(null)
            }
    }

    override fun getSession(result: (User?) -> Unit) {
        val user_str = appPreferences.getString(SharedPrefConstants.USER_SESSION,null)
        if (user_str == null){
            result.invoke(null)
        }else{
            val user = gson.fromJson(user_str,User::class.java)
            result.invoke(user)
        }
    }

    override fun withDrawal(result: () -> Unit) {
        auth.currentUser!!.delete().addOnCompleteListener{ task ->
            if(task.isSuccessful){
                auth.signOut()
                appPreferences.edit().putString(SharedPrefConstants.USER_SESSION,null).apply()
                result.invoke()
            }
        }
    }

    override fun deleteUserInfo(result: (UiState<String>) -> Unit) {
        var document = database.collection(FireStoreCollection.USER).document(ApplicationClass.prefs.userSeq.toString())
        document.delete()
            .addOnSuccessListener {
                result.invoke(
                    UiState.Success("User has been deleted successfully")
                )
            }
            .addOnFailureListener {
                result.invoke(
                    UiState.Failure(
                        it.localizedMessage
                    )
                )
            }

        if(ApplicationClass.prefs.isEachProvider == true){
            document = database.collection(FireStoreCollection.PROVIDER).document(ApplicationClass.prefs.providerId.toString())
            document.delete()
                .addOnSuccessListener {
                    result.invoke(
                        UiState.Success("User has been deleted successfully")
                    )
                }
                .addOnFailureListener {
                    result.invoke(
                        UiState.Failure(
                            it.localizedMessage
                        )
                    )
                }
        }
    }
}