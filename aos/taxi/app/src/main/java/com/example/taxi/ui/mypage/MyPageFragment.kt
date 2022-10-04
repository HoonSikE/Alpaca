package com.example.taxi.ui.mypage

import android.util.Log
import android.view.View
import androidx.core.content.ContextCompat
import androidx.core.os.bundleOf
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.bumptech.glide.Glide
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.dto.provider.TaxiUser
import com.example.taxi.data.dto.provider.UserList
import com.example.taxi.data.dto.user.User
import com.example.taxi.data.dto.user.boarded_taxi_list.BoardedTaxi
import com.example.taxi.data.dto.user.boarded_taxi_list.BoardedTaxiList
import com.example.taxi.databinding.FragmentMyPageBinding
import com.example.taxi.di.ApplicationClass
import com.example.taxi.ui.driving.end.EndDrivingViewModel
import com.example.taxi.ui.home.provider.ProviderAdapter
import com.example.taxi.ui.login.AuthViewModel
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.view.toast
import com.google.firebase.firestore.FirebaseFirestore
import dagger.hilt.android.AndroidEntryPoint

@AndroidEntryPoint
class MyPageFragment : BaseFragment<FragmentMyPageBinding>(R.layout.fragment_my_page) {
    val authViewModel: AuthViewModel by viewModels()
    private val endDrivingViewModel : EndDrivingViewModel by viewModels()

    private lateinit var myPageAdapter: MyPageAdapter
    private lateinit var boardedTaxiList: BoardedTaxiList

    private val onClickListener: (View, Int) -> Unit = { _, idx ->
        findNavController().navigate(R.id.action_myPageFragment_to_boardingListFragment)
    }

    override fun init() {
        initData()
        setOnClickListeners()
        observerData()
    }

    private fun initData() {
        val profileImage = ApplicationClass.prefs?.profileImage
        if (profileImage != "")
            Glide.with(this).load(profileImage).into(binding.imageMyPageProfile)

        binding.textMyPageName.text = ApplicationClass.prefs.name + "님, 안녕하세요!"
        binding.textMyPageCount.text = ApplicationClass.prefs.useCount.toString()

        val useCount = ApplicationClass.prefs.useCount
        if (useCount != null) {
            getGrade(useCount)
        }

        endDrivingViewModel.getBoardedTaxiList()
    }

    private fun setOnClickListeners() {
        binding.imageMyPageBack.setOnClickListener{
            requireActivity().onBackPressed()
        }
        binding.textMyPageFavorites.setOnClickListener{
            findNavController().navigate(R.id.action_myPageFragment_to_favoritesFragment)
        }
        binding.textUpdateUserInfo.setOnClickListener {
            findNavController().navigate(R.id.action_myPageFragment_to_updateUserInfoFragment)
        }
        binding.textUpdatePassword.setOnClickListener {
            findNavController().navigate(R.id.action_myPageFragment_to_updatePasswordFragment)
        }
        binding.textUpdateUserLogout.setOnClickListener {
            authViewModel.logout {
                findNavController().navigate(R.id.action_myPageFragment_to_loginFragment)
            }
        }
        binding.textUserWithdrawal.setOnClickListener {
            findNavController().navigate(R.id.action_myPageFragment_to_userWithdrawalFragment)
        }
        binding.imageMyPageChatBot.setOnClickListener{
            findNavController().navigate(R.id.action_myPageFragment_to_chatBotFragment)
        }
    }

    private fun observerData() {
        endDrivingViewModel.boardedTaxiList.observe(viewLifecycleOwner) {state ->
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
                    //binding.progressBar.hide()
//                    boardedTaxiList = state.data.taxiList as MutableList<BoardedTaxi>
                    boardedTaxiList = state.data
                    initAdapter(boardedTaxiList)
                }
            }
        }
    }

    private fun getGrade(useCount : Int){
        if(0 <= useCount && useCount < 5) {
            binding.textMyPageClass.setText("Bronze")
            binding.textMyPageClass.setTextColor(ContextCompat.getColor(requireContext(),R.color.bronze))
        }else if(useCount < 10){
            binding.textMyPageClass.setText("Siver")
            binding.textMyPageClass.setTextColor(ContextCompat.getColor(requireContext(), R.color.silver))
        }else if(useCount < 20){
            binding.textMyPageClass.setText("Gold")
            binding.textMyPageClass.setTextColor(ContextCompat.getColor(requireContext(), R.color.gold))
        }else if(useCount < 30){
            binding.textMyPageClass.setText("Platinum")
            binding.textMyPageClass.setTextColor(ContextCompat.getColor(requireContext(), R.color.platinum))
        }else if(useCount >= 30){
            binding.textMyPageClass.setText("Diamond")
            binding.textMyPageClass.setTextColor(ContextCompat.getColor(requireContext(), R.color.diamond))
        }
    }

    private fun initAdapter(TaxiList: BoardedTaxiList) {
        myPageAdapter = MyPageAdapter().apply {
            onBoaredTaxiClickListener = onClickListener
            context = requireContext()
            boardedTaxiList = TaxiList.taxiList as MutableList<BoardedTaxi>
        }
        binding.recyclerviewMyPageBoardTaxiList.apply {
            adapter = myPageAdapter
            layoutManager = LinearLayoutManager(requireContext(), RecyclerView.HORIZONTAL, false)
        }
    }
}