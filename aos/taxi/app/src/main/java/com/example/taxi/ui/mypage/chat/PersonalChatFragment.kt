package com.example.taxi.ui.mypage.chat

import android.os.Handler
import android.util.Log
import androidx.fragment.app.viewModels
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.mypage.Chat
import com.example.taxi.data.dto.mypage.ChatModel
import com.example.taxi.data.dto.user.boarded_taxi_list.BoardedTaxi
import com.example.taxi.databinding.FragmentPersonalChatBinding
import com.example.taxi.di.ApplicationClass
import com.example.taxi.ui.mypage.favorites.UpdateFavoritesAdapter
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.constant.hide
import com.example.taxi.utils.constant.show
import com.example.taxi.utils.view.toast
import com.google.firebase.database.DataSnapshot
import com.google.firebase.database.DatabaseError
import com.google.firebase.database.FirebaseDatabase
import com.google.firebase.database.ValueEventListener
import com.google.firebase.database.ktx.getValue
import dagger.hilt.android.AndroidEntryPoint
import org.w3c.dom.Comment
import java.text.SimpleDateFormat
import java.util.*

@AndroidEntryPoint
class PersonalChatFragment : BaseFragment<FragmentPersonalChatBinding>(R.layout.fragment_personal_chat) {
    private val personalChatViewModel : PersonalChatViewModel by viewModels()

    private lateinit var personalChatAdapter : PersonalChatAdapter
    private var comments = ArrayList<ChatModel.CommentModel>()
    private var chatRoomUid : String? = null
    private var startUserName : String? = null
    private var destinationUserImg : String? = null
    private var destinationUserName : String? = null

    override fun init() {
        initAdapter()
        initData()
        setOnClickListeners()
        observerData()
    }

    private fun initAdapter() {
        personalChatAdapter = PersonalChatAdapter()

        binding.messageActivityRecyclerview.apply {
            adapter = personalChatAdapter
            layoutManager = LinearLayoutManager(requireContext(), RecyclerView.VERTICAL, false)
        }
    }

    private fun initData() {
        startUserName = getName(arguments?.getString("startUserName") as String)
        destinationUserName = getName(arguments?.getString("destinationUserName") as String)
        ApplicationClass.prefs.destinationUserName = destinationUserName
        destinationUserImg = arguments?.getString("destinationUserImg") as String
        ApplicationClass.prefs.destinationUserImg = destinationUserImg
        binding.textPersonalChatUserName.text = destinationUserName

        personalChatViewModel.checkChatRoom(
            startUserName = startUserName!!,
            destinationUserName = destinationUserName!!
        )
    }
    private fun createChatRooms() {
        if (chatRoomUid == null) {
            println("        if (chatRoomUid == null)")
            val chatModel = ChatModel()
            chatModel.users.put(startUserName.toString(), true)
            chatModel.users.put(destinationUserName.toString(), true)

            personalChatViewModel.createChatRooms(
                chatModel = chatModel
            )
        }else{
            // 데이터 조회
            personalChatViewModel.getComment(
                destinationUserName = destinationUserName!!,
                chatRoomUid = chatRoomUid!!
            )
        }
    }

    private fun setOnClickListeners() {
        binding.imagePersonalChatBack.setOnClickListener{
            requireActivity().onBackPressed()
        }
        binding.imagePersonalChat.setOnClickListener{

            if(binding.edittextPersonalChat.text.toString() != "") {
                binding.imagePersonalChat.setAlpha(128)
                binding.imagePersonalChat.isEnabled = false

                println("chatRoomUid: " + chatRoomUid)
                //메세지 보내기
                // 시간 세팅
                val time = System.currentTimeMillis()
                val dateFormat = SimpleDateFormat("MM월dd일 hh:mm")
                val curTime = dateFormat.format(Date(time)).toString()

                val comment = ChatModel.CommentModel(
                    startUserName,
                    binding.edittextPersonalChat.text.toString(),
                    curTime
                )
                // 데이터 추가
                Handler().postDelayed({
                    personalChatViewModel.addComment(
                        chatRoomUid = chatRoomUid!!,
                        comment = comment
                    )
                    binding.edittextPersonalChat.text = null
                    binding.imagePersonalChat.isEnabled = true
                    binding.imagePersonalChat.setAlpha(255)
                }, 1000L)

                // 데이터 조회
                personalChatViewModel.getComment(
                    destinationUserName = destinationUserName!!,
                    chatRoomUid = chatRoomUid!!
                )
            }else{
                toast("메시지를 입력해주세요.")
            }
        }
    }

    private fun observerData() {
        personalChatViewModel.createChatRooms.observe(viewLifecycleOwner){ state ->
            when(state){
                is UiState.Loading ->{

                }
                is UiState.Failure -> {
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                }
                is UiState.Success -> {
                    personalChatViewModel.checkChatRoom(
                        startUserName = startUserName!!,
                        destinationUserName = destinationUserName!!
                    )
                }
            }
        }

        personalChatViewModel.checkChatRoom.observe(viewLifecycleOwner){ state ->
            when(state){
                is UiState.Loading ->{

                }
                is UiState.Failure -> {
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                }
                is UiState.Success -> {
                    for(data in state.data.children){
                        val item = data.getValue<ChatModel>()

                        if(item?.users!!.containsKey(destinationUserName)){
                            chatRoomUid = data.key
                            println("chatRoomUid: " + chatRoomUid)
                        }
                    }
                    createChatRooms()
                }
            }
        }

        personalChatViewModel.getComment.observe(viewLifecycleOwner){ state ->
            when(state){
                is UiState.Loading ->{

                }
                is UiState.Failure -> {
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                }
                is UiState.Success -> {
                    comments.clear()
                    for(data in state.data.children){
                        val item = data.getValue<ChatModel.CommentModel>()
                        comments.add(item!!)
                        println(comments)
                    }
                    personalChatAdapter.updateList(comments)
                }
            }
        }
    }
    private fun getName(userName: String) : String {
        var result = userName
        // email Set으로 들어가면 오류가 생기므로 .과 @를 없앤다.
        result = result!!.replace(".", "")
        result = result!!.replace("@", "")
        return result
    }
}
