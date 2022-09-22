package com.example.taxi.ui.mypage.update_user

import android.Manifest
import android.app.Activity
import android.content.Intent
import android.content.pm.PackageManager
import android.net.Uri
import android.util.Log
import androidx.core.content.ContextCompat
import androidx.fragment.app.viewModels
import com.bumptech.glide.Glide
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.user.User
import com.example.taxi.databinding.FragmentUpdateUserInfoBinding
import com.example.taxi.di.ApplicationClass
import com.example.taxi.utils.constant.FireStoreCollection.USER
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.view.toast
import com.google.firebase.firestore.FirebaseFirestore
import com.google.firebase.storage.FirebaseStorage
import dagger.hilt.android.AndroidEntryPoint
import de.hdodenhof.circleimageview.CircleImageView
import java.text.SimpleDateFormat
import java.util.*

@AndroidEntryPoint
class UpdateUserInfoFragment : BaseFragment<FragmentUpdateUserInfoBinding>(R.layout.fragment_update_user_info) {

    private val updateUserInfoViewModel : UpdateUserInfoViewModel by viewModels()
    // 사진 불러오기
    var firestore = FirebaseFirestore.getInstance()
//    var storage = FirebaseStorage.getInstance("gs://taxi-9f12e.appspot.com")
    // 사진 업로드
    var pickImageFromAlbum = 0
    var fbStorage : FirebaseStorage? = FirebaseStorage.getInstance()
    var uriPhoto : Uri? = null

    override fun init() {
        initData()
        setOnClickListeners()
        observerData()
    }

    private fun initData(){
        if(ApplicationClass.userId != null){
            firestore.collection("User").document(ApplicationClass.prefs.userSeq.toString()).get().addOnCompleteListener{ task ->
                if(task.isSuccessful){
                    var user = task.result?.toObject(User::class.java)
                    // 프로필 이미지가 없다면 기본 이미지 출력
                    if(user?.profileImage != "")
                        Glide.with(this).load(user?.profileImage).into(binding.imageUpdateUserImage)
                }
            }
        }
        binding.imageUpdateUserImage.setImageURI(uriPhoto)

        binding.textUpdateUserInfoName.text = ApplicationClass.prefs.name
        binding.textUpdateUserInfoEmail.text = ApplicationClass.userId
        binding.textUpdateUserInfoPhone.text = ApplicationClass.prefs.tel
        updateUserInfoViewModel.getAddressInfo()
    }

    private fun setOnClickListeners(){
        binding.imageUpdateUserImageButton.setOnClickListener{
            // Open Album
            var photoPickerInent = Intent(Intent.ACTION_PICK)
            photoPickerInent.type = "image/*"
            startActivityForResult(photoPickerInent, pickImageFromAlbum)
        }
        binding.textUpdateUserInfoPhone.setOnClickListener{
            //TODO: 휴대폰 번호 수정
        }
        binding.textUpdateUserInfoHomeAddress.setOnClickListener{
            //TODO: 집 주소 수정
        }
        binding.textUpdateUserInfoOfficeAddress.setOnClickListener{
            //TODO: 회사 주소 수정
        }
    }

    private fun observerData(){
        updateUserInfoViewModel.addressInfo.observe(viewLifecycleOwner){ state ->
            when (state) {
                is UiState.Loading -> {
                    //binding.progressBar.show()
                }
                is UiState.Failure -> {
                    //binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                }
                is UiState.Success -> {
                    binding.textUpdateUserInfoHomeAddress.text = state.data.home
                    binding.textUpdateUserInfoOfficeAddress.text = state.data.company
                    println("test: " + state.data.home)
                }
            }
        }
    }

    // 앨범에서 사진을 선택할 시 Firebase Storage에 업로드
    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?){
        super.onActivityResult(requestCode, resultCode, data)

        if(requestCode == pickImageFromAlbum){
            if(resultCode == Activity.RESULT_OK){
                // 앨범 사진 출력
                uriPhoto = data?.data
                binding.imageUpdateUserImage.setImageURI(uriPhoto)

//                // 권한 확인 후 업로드
//                if(ContextCompat.checkSelfPermission(binding.imageUpdateUserImage!!.context,
//                        Manifest.permission.READ_EXTERNAL_STORAGE) == PackageManager.PERMISSION_GRANTED){
                    funImageUpload(binding.imageUpdateUserImage!!)
//                }
            }
        }
    }

    private fun funImageUpload(view : CircleImageView){
        var userSeq = ApplicationClass.prefs.userSeq.toString()
        var imgFileName = userSeq + ".png"
        var storage = FirebaseStorage.getInstance()

        storage.getReference().child("user_profiles").child(imgFileName)
            .putFile(uriPhoto!!)//어디에 업로드할지 지정
            .addOnSuccessListener {
                    taskSnapshot -> // 업로드 정보를 담는다
                taskSnapshot.metadata?.reference?.downloadUrl?.addOnSuccessListener {
                        it->
                    var imageUrl=it.toString()

                    firestore.collection("User")
                        .document(userSeq).update("profileImage", imageUrl)
                        .addOnSuccessListener {
                            toast("Success Image Uploaded")
                        }.addOnFailureListener{
                             toast("Failed Image Uploaded")
                        }
                }
            }
    }
}