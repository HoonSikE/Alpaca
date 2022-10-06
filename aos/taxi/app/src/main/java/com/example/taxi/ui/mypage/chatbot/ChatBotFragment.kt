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
    private var message = "0"
    private var preMsg = Stack<String>()
    private var messageTmp = "-1"

    private val messageClickListener: (View, String) -> Unit = { _, msg ->
        val tmp = msg.split(".")

        if(message != tmp[0] && msg.substring(0 until 2) != "아래"){
            if(preMsg.size > 0){
                if(preMsg.peek() != messageTmp){
                    preMsg.push(messageTmp)
                }
            }
            messageTmp = msg

            message = tmp[0]
            println("click message: " + msg)
            println("message: " + tmp[0])

            chatBotViewModel.checkChatBotMessage(
                message = message
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
        chatBotViewModel.checkChatBotMessage(
            message = message
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
                        if(messageTmp == "-1"){
                            comments.add("안녕하세요! 저는 알파카 챗봇이라고 해요!")
                        } else {
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