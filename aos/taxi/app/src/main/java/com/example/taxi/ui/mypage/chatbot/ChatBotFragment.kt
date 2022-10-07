package com.example.taxi.ui.mypage.chatbot

import android.util.Log
import android.view.View
import androidx.fragment.app.viewModels
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.databinding.FragmentChatBotBinding
import com.example.taxi.di.ApplicationClass
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.constant.hide
import com.example.taxi.utils.constant.show
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint
import java.util.*

@AndroidEntryPoint
class ChatBotFragment : BaseFragment<FragmentChatBotBinding>(R.layout.fragment_chat_bot) {
    private val chatBotViewModel: ChatBotViewModel by viewModels()
    private lateinit var chatBotAdapter: ChatBotAdapter
    private var comments = ArrayList<String>()
    private var lastMessage = "0"
    private var preMsg = Stack<String>()
    private var messageTmp = "0"

    private val messageClickListener: (View, String) -> Unit = { _, msg ->
        if(preMsg.size > 0){
            println("뭐냐뭐냐: " + preMsg.peek())
        }
        val tmp = msg.split(".")

        if(lastMessage != tmp[0] && msg.substring(0 until 2) != "아래"){
            // 중복되게 넣지 않음.
            if(preMsg.size > 0) {
                if (preMsg.peek() != msg) {
                    preMsg.push(msg)
                }
            } else if(preMsg.size == 0){
                preMsg.push(messageTmp)
            }
            // 사용자에게 출력 메세지
            messageTmp = msg

            // 챗봇 메세지
            lastMessage = tmp[0]
            println("click message: " + msg)
            println("message: " + tmp[0])

            chatBotViewModel.checkChatBotMessage(
                message = lastMessage
            )
        }
    }

    override fun init() {
        initAdapter()
        initData()
        setOnClickListeners()
        observerData()
    }

    private fun initAdapter() {
        chatBotAdapter = ChatBotAdapter().apply {
            onMessageClickListener = messageClickListener
        }

        binding.messageActivityRecyclerview.apply {
            adapter = chatBotAdapter
            layoutManager = LinearLayoutManager(requireContext(), RecyclerView.VERTICAL, false)
        }
    }

    private fun initData() {
        comments.add("안녕하세요! 저는 알파카 챗봇이라고 해요!")
        chatBotViewModel.checkChatBotMessage(
            message = lastMessage
        )
    }

    private fun setOnClickListeners() {
        binding.imgChatBotBack.setOnClickListener{
            requireActivity().onBackPressed()
        }
        binding.buttonChatBotStart.setOnClickListener{
            preMsg.clear()
            messageTmp = "0. 처음으로"
            chatBotViewModel.checkChatBotMessage(
                message = "0"
            )
        }
        binding.buttonChatBotPrevious.setOnClickListener{
            if(preMsg.size > 0){
                messageTmp = preMsg.pop()
                var tmp = messageTmp.split(".")
                chatBotViewModel.checkChatBotMessage(
                    message = tmp[0]
                )
            }
        }
        binding.buttonChatBotCall.setOnClickListener{
            preMsg.clear()
            messageTmp = "0-0. 상담사 연결"
            chatBotViewModel.checkChatBotMessage(
                message = "0-0"
            )
        }
    }

    private fun observerData() {
        chatBotViewModel.checkChatBotMessage.observe(viewLifecycleOwner){ state ->
            when(state){
                is UiState.Loading ->{
                    binding.progressBar.show()
                }
                is UiState.Failure -> {
                    binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                }
                is UiState.Success -> {
                    binding.progressBar.hide()
                    if(state.data.value != null){
                        if(comments.size > 1){
                            comments.add("나:" + messageTmp)
                        }
                        comments.add("아래 항목을 선택해주세요!")
                    }
                    for(data in state.data.children){
                        val item = data
                        comments.add(item.value.toString())
                        println(comments)
                    }
                    chatBotAdapter.updateList(comments)
                    binding.messageActivityRecyclerview.scrollToPosition(chatBotAdapter.itemCount-1)
                    println("comments : " + comments)
                }
                else -> {}
            }
        }
    }
}